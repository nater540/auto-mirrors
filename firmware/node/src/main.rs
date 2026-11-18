//! Mirror node firmware for the ESP32-C6 (milestone M4).
//!
//! The decisions are in `node-logic`, tested on the host; this file wires them to the hardware.
//! Every pin is in `board.rs`, which is PROVISIONAL until the node board exists.
//!
//! Tasks:
//! - `inputs`: samples the permit and rail inputs every millisecond, and runs the node's clock (heartbeat, status).
//! - `arm`: toggles the arm pulse at 1 kHz while the node says a move may run, and holds it low otherwise.
//!   It is software on purpose: a hung CPU stops the pulse, and the charge pump then drops the rail.
//! - `can`: receives and decodes frames for the node, and sends what it queues. Recovers from bus-off.
//! - `actuator`: owns the steppers and the flash. Runs one move at a time, and writes position records only
//!   between moves, so that no flash write happens while a motor steps (hand-off 3.3).
//! - `console`: the bench console over USB-Serial-JTAG; replies go to the log.
//!
//! `FLASH_NOTICE.txt`, beside this crate's `Cargo.toml`, states what the image does to motors and I/O.
//! `just flash` prints it and asks before flashing, and the image logs it at boot.
//! Change it in the same commit as any change to what this firmware drives.
#![no_std]
#![no_main]
#![deny(
  clippy::mem_forget,
  reason = "mem::forget is not safe with esp-hal types that hold a buffer for the length of a transfer"
)]
#![deny(clippy::large_stack_frames)]

mod board;

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, Ordering};

use align_core::config::Config;
use align_proto::{Channel, Envelope};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as Queue;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer, with_timeout};
use embedded_can::Frame as _;
use embedded_io_async::Read as _;
use esp_backtrace as _;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::peripherals::TIMG1;
use esp_hal::time::Duration as HalDuration;
use esp_hal::timer::timg::{MwdtStage, TimerGroup, Wdt};
use esp_hal::twai::{BaudRate, EspTwaiFrame, Twai};
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx};
use esp_storage::FlashStorage;
use log::{error, info, warn};
use node_logic::bench_line::{BenchEnd, BenchLine};
use node_logic::console::{self, Command};
use node_logic::{AxisRecord, MAX_CHANNELS, MoveOutcome, MoveRequest, Node, NodeIo, NodeSetup, Restored};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage, SerializationError, Value};
use static_cell::StaticCell;
use stepper::{Coils, MoveEnd, Phase, Stepper};

/// What this image does to motors and I/O: the text `just flash` shows before flashing.
const FLASH_NOTICE: &str = include_str!("../FLASH_NOTICE.txt");

// The app descriptor the ESP-IDF second-stage bootloader requires in every image.
esp_bootloader_esp_idf::esp_app_desc!();

/// Firmware version, reported in the heartbeat.
const FW_VERSION: u8 = 1;

/// 250 kbit/s (reference 5.3; an UNVERIFIED choice for a 2 to 3 m drag-chain bus).
const BAUD_RATE: BaudRate = BaudRate::B250K;

type Raw = CriticalSectionRawMutex;
type Coil = Output<'static>;
type Flash = BlockingAsync<FlashStorage<'static>>;
type Records = MapStorage<u8, Flash, Cache<Uncached, Uncached, Uncached, u8>>;

static NODE: Mutex<Raw, RefCell<Option<Node>>> = Mutex::new(RefCell::new(None));
static FRAMES_OUT: Queue<Raw, Envelope, 16> = Queue::new();
static MOVES: Queue<Raw, MoveRequest, 2> = Queue::new();
/// The newest record of each channel not yet written. A newer record replaces an older one of its channel, which
/// only the newest matters for, so no burst of records can overflow it and drop the one marked suspect.
static PENDING_RECORDS: Mutex<Raw, RefCell<[Option<AxisRecord>; MAX_CHANNELS]>> =
  Mutex::new(RefCell::new([None; MAX_CHANNELS]));
static RECORD_PENDING: Signal<Raw, ()> = Signal::new();
static STOP: AtomicBool = AtomicBool::new(false);
static RAIL_OK: AtomicBool = AtomicBool::new(false);
/// The permit input as last sampled, for the console's `status`; the node keeps its own copy.
static PERMIT: AtomicBool = AtomicBool::new(false);
static MOVE_FINISHED: Signal<Raw, MoveOutcome> = Signal::new();

/// The firmware side of [`NodeIo`]: queues and flags the tasks serve.
struct Outbox;

impl NodeIo for Outbox {
  fn send(&mut self, envelope: Envelope) {
    if FRAMES_OUT.try_send(envelope).is_err() {
      warn!("CAN transmit queue full: dropped {:?}", envelope.message);
    }
  }

  fn start_move(&mut self, request: MoveRequest) {
    // Cleared here, before the request is queued, so that a stop requested right after cannot be lost.
    STOP.store(false, Ordering::Release);
    if MOVES.try_send(request).is_err() {
      error!(
        "move queue full: the move on channel {} was not started",
        request.channel.get()
      );
    }
  }

  fn stop_move(&mut self) {
    STOP.store(true, Ordering::Release);
  }

  fn persist(&mut self, channel: Channel, record: AxisRecord) {
    let queued = PENDING_RECORDS.lock(|cell| {
      let mut pending = cell.borrow_mut();
      let slot = pending.get_mut(usize::from(channel.get()))?;
      *slot = Some(record);
      Some(())
    });
    if queued.is_none() {
      error!(
        "channel {} has no record slot: its record was not written",
        channel.get()
      );
    }
    RECORD_PENDING.signal(());
  }
}

/// The oldest-numbered channel with a record waiting to be written, and that record.
fn take_pending_record() -> Option<(Channel, AxisRecord)> {
  PENDING_RECORDS.lock(|cell| {
    let mut pending = cell.borrow_mut();
    (0_u8..).zip(pending.iter_mut()).find_map(|(index, slot)| {
      let channel = Channel::new(index)?;
      Some((channel, slot.take()?))
    })
  })
}

/// Runs `f` on the node. `None` before the node exists, or if it is already borrowed (which would be a bug here:
/// [`Outbox`] never calls back into the node).
fn with_node<R>(f: impl FnOnce(&mut Node, &mut Outbox) -> R) -> Option<R> {
  NODE.lock(|cell| cell.try_borrow_mut().ok()?.as_mut().map(|node| f(node, &mut Outbox)))
}

fn now_ms() -> u64 {
  Instant::now().as_millis()
}

/// A position record as sequential-storage stores it.
struct StoredRecord(AxisRecord);

impl Value<'_> for StoredRecord {
  fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
    let bytes = self.0.to_bytes();
    buffer
      .get_mut(..bytes.len())
      .ok_or(SerializationError::BufferTooSmall)?
      .copy_from_slice(&bytes);
    Ok(bytes.len())
  }

  fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
    let record = AxisRecord::from_bytes(buffer).map_err(|_| SerializationError::InvalidData)?;
    Ok((Self(record), buffer.len()))
  }
}

/// Longest console line; a longer one is discarded rather than run truncated.
const LINE_LEN: usize = 80;

/// A warning that repeats, logged at most once a second with a count of what it held back.
struct RateLimited {
  last_ms: Option<u64>,
  held_back: u32
}

impl RateLimited {
  const fn new() -> Self {
    Self {
      last_ms: None,
      held_back: 0
    }
  }

  /// Whether to log now, and how many were held back since the last one.
  fn admit(&mut self, now_ms: u64) -> Option<u32> {
    if self.last_ms.is_some_and(|last| now_ms.saturating_sub(last) < 1_000) {
      self.held_back = self.held_back.saturating_add(1);
      return None;
    }
    self.last_ms = Some(now_ms);
    Some(core::mem::take(&mut self.held_back))
  }
}

/// Room for one key and one record, rounded up to the flash word.
const STORAGE_BUFFER_LEN: usize = 32;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
  esp_println::logger::init_logger_from_env();

  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
  let settings = Config::default();

  for line in FLASH_NOTICE.lines() {
    info!("{line}");
  }

  // Outputs first, all low: no coil and no arm pulse before anything else runs.
  let pins = board_pins!(peripherals);
  let low = |pin| Output::new(pin, Level::Low, OutputConfig::default());
  let coils = pins.channels.map(|channel| channel.map(low));
  let arm = low(pins.arm);
  let pulled_down = InputConfig::default().with_pull(Pull::Down);
  let permit = Input::new(pins.permit, pulled_down);
  let rail = Input::new(pins.rail_sense, pulled_down);

  let Some(mut records) = open_records(FlashStorage::new(peripherals.FLASH)) else {
    halt("no usable flash partition for position records: refusing to run the motors").await
  };
  let restored = restore(&mut records).await;
  for (channel, restored) in restored.iter().enumerate() {
    info!("channel {channel}: {restored:?}");
  }
  let phase = |channel: usize| match restored.get(channel) {
    Some(Restored::Valid(record)) => record.phase,
    _ => Phase::ZERO
  };
  let mut channel = 0_usize;
  let steppers = coils.map(|coils| {
    let stepper = Stepper::new(
      Coils::new(coils).unwrap_or_else(|never| match never.0 {}),
      phase(channel)
    );
    channel = channel.saturating_add(1);
    stepper
  });

  let mut node = Node::new(NodeSetup {
    id: board::NODE_ID,
    populated: board::POPULATED_CHANNELS,
    fw_version: FW_VERSION,
    motion: settings.motion,
    timing: settings.node
  });
  node.restore(restored);
  NODE.lock(|cell| cell.replace(Some(node)));

  let twai = board::twai(peripherals.TWAI0, pins.twai_tx, pins.twai_rx, BAUD_RATE)
    .into_async()
    .start();
  let (usb_rx, _usb_tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async().split();

  // Each task is spawned once into a pool of one, so creating its token cannot fail; if it ever did, the task would
  // be missing and the node would refuse to move (no tick, no actuator), which the log names.
  macro_rules! spawn {
    ($task:expr) => {
      match $task {
        Ok(token) => spawner.spawn(token),
        Err(error) => error!("spawning `{}` failed: {error:?}", stringify!($task))
      }
    };
  }
  // The watchdog resets the chip, and with it every coil output, if the input loop stops feeding it: a panic
  // spins with interrupts masked, and a hang stops the loop. It starts only now, with the loop that feeds it:
  // before this no coil can be driven, while erasing the record partition at boot may take longer than its
  // timeout (six sectors, up to 400 ms each by flash datasheets), and a halt above must stay halted.
  let mut watchdog = TimerGroup::new(peripherals.TIMG1).wdt;
  watchdog.set_timeout(
    MwdtStage::Stage0,
    HalDuration::from_millis(u64::from(settings.node.watchdog_timeout_ms))
  );
  watchdog.enable();
  spawn!(inputs(permit, rail, watchdog));
  spawn!(arm_pulse(arm));
  spawn!(can(twai));
  spawn!(actuator(steppers, records, settings.node.rail_up_timeout_ms));
  spawn!(console_task(usb_rx));
  info!(
    "node {} ready; `help` on this console lists the bench commands",
    board::NODE_ID.get()
  );

  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}

/// Logs the reason and parks forever with every output as configured at boot (low).
async fn halt(reason: &str) -> ! {
  error!("{reason}");
  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}

/// The record store on the `nvs` data partition, which this firmware uses for nothing else.
fn open_records(mut flash: FlashStorage<'static>) -> Option<Records> {
  use esp_bootloader_esp_idf::partitions::{
    DataPartitionSubType, PARTITION_TABLE_MAX_LEN, PartitionType, read_partition_table
  };
  let mut table_buffer = [0_u8; PARTITION_TABLE_MAX_LEN];
  let table = read_partition_table(&mut flash, &mut table_buffer).ok()?;
  let partition = table
    .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
    .ok()??;
  let start = partition.offset();
  let range = start..start.checked_add(partition.len())?;
  info!("position records in flash {:#x}..{:#x}", range.start, range.end);
  let config = MapConfig::try_new(range).ok()?;
  Some(MapStorage::new(
    BlockingAsync::new(flash),
    config,
    Cache::new_uncached()
  ))
}

/// Reads every channel's record. Corrupt storage (for example, ESP-IDF NVS data left by an earlier image) is
/// erased, and every channel then starts fresh but suspect.
async fn restore(records: &mut Records) -> [Restored; MAX_CHANNELS] {
  let mut restored = [Restored::Missing; MAX_CHANNELS];
  let mut corrupt = false;
  for (key, slot) in (0_u8..).zip(restored.iter_mut()) {
    let mut buffer = [0_u8; STORAGE_BUFFER_LEN];
    match records.fetch_item::<StoredRecord>(&mut buffer, &key).await {
      Ok(Some(StoredRecord(record))) => *slot = Restored::Valid(record),
      Ok(None) => {}
      Err(sequential_storage::Error::SerializationError(_)) => *slot = Restored::Corrupt,
      Err(other) => {
        warn!("reading the record of channel {key} failed: {other:?}");
        corrupt = true;
      }
    }
  }
  if corrupt {
    warn!("erasing the record partition; every channel starts at zero and suspect");
    if let Err(error) = records.erase_all().await {
      error!("erasing the record partition failed: {error:?}");
    }
    restored = [Restored::Corrupt; MAX_CHANNELS];
    // Written at once, so that a second reboot before any move still finds the channels suspect, not fresh.
    let suspect = AxisRecord {
      suspect: true,
      ..AxisRecord::FRESH
    };
    for key in (0_u8..).take(MAX_CHANNELS) {
      let mut buffer = [0_u8; STORAGE_BUFFER_LEN];
      if let Err(error) = records.store_item(&mut buffer, &key, &StoredRecord(suspect)).await {
        error!("writing the suspect record of channel {key} failed: {error:?}");
      }
    }
  }
  restored
}

#[embassy_executor::task]
async fn inputs(permit: Input<'static>, rail: Input<'static>, mut watchdog: Wdt<TIMG1<'static>>) {
  let mut ticker = Ticker::every(Duration::from_millis(1));
  loop {
    watchdog.feed();
    let rail_ok = rail.is_high();
    RAIL_OK.store(rail_ok, Ordering::Release);
    let permit_present = permit.is_high();
    PERMIT.store(permit_present, Ordering::Release);
    let now = now_ms();
    with_node(|node, io| {
      node.set_permit(permit_present, io);
      node.set_rail_ok(rail_ok);
      node.tick(now, io);
    });
    ticker.next().await;
  }
}

#[embassy_executor::task]
async fn arm_pulse(mut arm: Output<'static>) {
  // Half of a 1 kHz period.
  let mut ticker = Ticker::every(Duration::from_micros(500));
  loop {
    if with_node(|node, _| node.armed(now_ms())).unwrap_or(false) {
      arm.toggle();
    } else {
      arm.set_low();
    }
    ticker.next().await;
  }
}

#[embassy_executor::task]
async fn can(mut twai: Twai<'static, Async>) {
  let mut rejected = RateLimited::new();
  // Alone on the bus (no coordinator powered), every frame fails: ten heartbeats a second.
  let mut unsent = RateLimited::new();
  loop {
    if twai.is_bus_off() {
      // Reference 5.3: brake all, wait 100 ms, recover once, and wait for the recovery to finish rather than
      // starting it again. Motors stay stopped until the coordinator allows them.
      warn!("CAN bus-off: stopping, recovering in 100 ms");
      with_node(Node::stop);
      Timer::after(Duration::from_millis(100)).await;
      twai.initiate_recovery();
      let recovered = with_timeout(Duration::from_secs(1), async {
        while twai.is_bus_off() {
          Timer::after(Duration::from_millis(5)).await;
        }
      });
      if recovered.await.is_err() {
        warn!("CAN bus-off: no recovery within 1 s; trying again");
      }
      continue;
    }
    match select(twai.receive_async(), FRAMES_OUT.receive()).await {
      Either::First(Ok(frame)) => {
        if frame.is_standard() && !frame.is_remote_frame() {
          match Envelope::decode(&frame) {
            Ok(envelope) => {
              let now = now_ms();
              with_node(|node, io| node.on_frame(now, envelope, io));
            }
            Err(error) => {
              if let Some(held_back) = rejected.admit(now_ms()) {
                warn!("CAN frame rejected: {error} ({held_back} more held back)");
              }
            }
          }
        }
      }
      Either::First(Err(error)) => warn!("CAN receive error: {error:?}"),
      Either::Second(envelope) => match envelope.encode::<EspTwaiFrame>() {
        Ok(frame) => {
          // `Err(TimeoutError)` when the 20 ms ran out, `Ok(Err(..))` when esp-hal gave up on the frame.
          let result = with_timeout(Duration::from_millis(20), twai.transmit_async(&frame)).await;
          if !matches!(result, Ok(Ok(())))
            && let Some(held_back) = unsent.admit(now_ms())
          {
            warn!(
              "CAN transmit failed: {result:?}, transmit errors {} ({held_back} more held back)",
              twai.transmit_error_count()
            );
          }
        }
        Err(error) => error!("encoding {:?} failed: {error}", envelope.message)
      }
    }
  }
}

#[embassy_executor::task]
async fn actuator(mut steppers: [Stepper<Coil>; board::CHANNELS], mut records: Records, rail_up_timeout_ms: u32) {
  loop {
    // Records first: they are written only here, between moves.
    while let Some((channel, record)) = take_pending_record() {
      write_record(&mut records, channel, record).await;
    }
    match select(RECORD_PENDING.wait(), MOVES.receive()).await {
      // The loop writes it.
      Either::First(()) => {}
      Either::Second(request) => {
        // Among them the record marked suspect that the node queued with this move: it is written before a step.
        while let Some((channel, record)) = take_pending_record() {
          write_record(&mut records, channel, record).await;
        }
        let outcome = match steppers.get_mut(usize::from(request.channel.get())) {
          Some(stepper) => run(stepper, request, rail_up_timeout_ms).await,
          None => MoveOutcome::Finished {
            end: MoveEnd::RailLost,
            steps_issued: 0,
            phase: Phase::ZERO
          }
        };
        info!("channel {}: {outcome:?}", request.channel.get());
        let now = now_ms();
        with_node(|node, io| node.on_move_outcome(now, outcome, io));
        MOVE_FINISHED.signal(outcome);
      }
    }
  }
}

async fn write_record(records: &mut Records, channel: Channel, record: AxisRecord) {
  let mut buffer = [0_u8; STORAGE_BUFFER_LEN];
  let result = records
    .store_item(&mut buffer, &channel.get(), &StoredRecord(record))
    .await;
  if let Err(error) = &result {
    error!("writing the record of channel {} failed: {error:?}", channel.get());
  }
  with_node(|node, io| node.on_persisted(channel, result.is_ok(), io));
}

/// Runs the legs of one move under its timeout. The rail must come up first: the arm pulse starts with the move.
async fn run(stepper: &mut Stepper<Coil>, request: MoveRequest, rail_up_timeout_ms: u32) -> MoveOutcome {
  // Waits for the rail; a stop ends the wait at once (the arm pulse stops with it, so the rail would never rise).
  let rail_up = async {
    loop {
      if STOP.load(Ordering::Acquire) {
        return false;
      }
      if RAIL_OK.load(Ordering::Acquire) {
        return true;
      }
      Timer::after(Duration::from_millis(1)).await;
    }
  };
  let end_before_stepping = match with_timeout(Duration::from_millis(u64::from(rail_up_timeout_ms)), rail_up).await {
    Ok(true) => None,
    Ok(false) => Some(MoveEnd::Stopped),
    Err(_) => Some(MoveEnd::RailLost)
  };
  if let Some(end) = end_before_stepping {
    return MoveOutcome::Finished {
      end,
      steps_issued: 0,
      phase: stepper.phase()
    };
  }
  // Steps of the legs already finished; the leg in progress counts its own on the stepper. Kept outside the
  // future, so that a timeout that drops it still knows how far the move got.
  let finished_legs = Cell::new(0_i32);
  let legs = async {
    for leg in request.legs_steps {
      let report = stepper
        .run(leg, request.timing, &mut Delay, &STOP, || {
          RAIL_OK.load(Ordering::Acquire)
        })
        .await
        .unwrap_or_else(|never| match never.0 {});
      finished_legs.set(finished_legs.get().saturating_add(report.steps_issued));
      if report.end != MoveEnd::Done {
        return report.end;
      }
    }
    MoveEnd::Done
  };
  match with_timeout(Duration::from_millis(u64::from(request.timeout_ms)), legs).await {
    Ok(end) => MoveOutcome::Finished {
      end,
      steps_issued: finished_legs.get(),
      phase: stepper.phase()
    },
    // The dropped move released its coils (the stepper's guard); the leg in progress still knows its steps.
    Err(_) => MoveOutcome::TimedOut {
      steps_issued: finished_legs.get().saturating_add(stepper.steps_issued()),
      phase: stepper.phase()
    }
  }
}

#[embassy_executor::task]
async fn console_task(mut rx: UsbSerialJtagRx<'static, Async>) {
  static LINE: StaticCell<[u8; LINE_LEN]> = StaticCell::new();
  let line = LINE.init([0; LINE_LEN]);
  let mut len = 0_usize;
  let mut overlong = false;
  loop {
    let mut byte = [0_u8; 1];
    if !matches!(rx.read(&mut byte).await, Ok(1)) {
      continue;
    }
    match byte[0] {
      b'\r' | b'\n' => {
        let text = line
          .get(..len)
          .and_then(|bytes| core::str::from_utf8(bytes).ok())
          .unwrap_or("");
        if overlong {
          warn!("line longer than {LINE_LEN} characters: discarded");
        } else if !text.trim().is_empty() {
          info!("> {text}");
          execute(text, &mut rx).await;
        }
        len = 0;
        overlong = false;
      }
      other => {
        if let Some(slot) = line.get_mut(len) {
          *slot = other;
          len += 1;
        } else {
          overlong = true;
        }
      }
    }
  }
}

async fn execute(text: &str, rx: &mut UsbSerialJtagRx<'static, Async>) {
  let settings = Config::default();
  match console::parse(text) {
    Err(error) => warn!("{error}"),
    Ok(Command::Help) => {
      for line in console::HELP.lines() {
        info!("{line}");
      }
    }
    Ok(Command::Status) => {
      with_node(|node, _| {
        let (permit, rail) = (PERMIT.load(Ordering::Acquire), RAIL_OK.load(Ordering::Acquire));
        info!("state {:?}, permit {permit}, rail {rail}", node.state());
        info!(
          "{}",
          BenchLine::Status {
            state: node.state(),
            permit,
            rail
          }
        );
        for channel in (0..board::POPULATED_CHANNELS).filter_map(Channel::new) {
          if let Some(axis) = node.axis(channel) {
            info!("channel {}: {:?}", channel.get(), axis);
            info!(
              "{}",
              BenchLine::Channel {
                channel: channel.get(),
                pos_steps: axis.record.pos_steps,
                suspect: axis.record.suspect
              }
            );
          }
        }
      });
    }
    Ok(Command::Bench(on)) => match with_node(|node, _| node.set_bench(on)) {
      Some(Ok(())) => info!("bench mode {}", if on { "on" } else { "off" }),
      Some(Err(error)) => warn!("{error}"),
      None => {}
    },
    Ok(Command::Stop) => {
      with_node(Node::stop);
    }
    Ok(Command::Zero { channel }) => {
      let Some(channel) = Channel::new(channel) else {
        warn!("no such channel");
        return;
      };
      match with_node(|node, io| node.bench_zero(channel, io)) {
        Some(Ok(())) => info!("channel {}: position set to zero", channel.get()),
        Some(Err(error)) => warn!("{error}"),
        None => {}
      }
    }
    Ok(Command::Jog {
      channel,
      steps,
      rate_pps
    }) => {
      let period = rate_pps.map_or(Some(settings.motion.step_period_us), console::period_us);
      bench_move(channel, steps, period, rx).await;
    }
    Ok(Command::Torque { channel, rate_pps }) => {
      let test = settings.torque_test;
      info!(
        "torque test: channel {channel}, {} cycles of +/-{} steps at {rate_pps} pps",
        test.cycles, test.amplitude_steps
      );
      let mut completed = 0_u32;
      for steps in console::torque_moves(test.amplitude_steps, test.cycles) {
        if !bench_move(channel, steps, console::period_us(rate_pps), rx).await {
          break;
        }
        completed += 1;
      }
      let total = 2 * u32::from(test.cycles);
      if completed == total {
        info!("torque test: all {total} moves completed; check the knob against its mark");
      } else {
        warn!("torque test: stopped after {completed} of {total} moves");
      }
    }
  }
}

/// Runs one bench move and waits for it. Any key stops it. Returns whether it completed.
///
/// Ends with one [`BenchLine`] for a program at the console: the move's end and the count after it, or its refusal.
async fn bench_move(channel: u8, steps: i32, period_us: Option<u32>, rx: &mut UsbSerialJtagRx<'static, Async>) -> bool {
  let (Some(channel), Some(period_us)) = (Channel::new(channel), period_us) else {
    warn!("no such channel or rate");
    // A rate of zero, or a channel past the bus's eight, both checked again by the node.
    info!(
      "{}",
      BenchLine::Refused {
        channel,
        reason: node_logic::BenchRefused::OutOfBounds
      }
    );
    return false;
  };
  MOVE_FINISHED.reset();
  match with_node(|node, io| node.bench_move(channel, steps, period_us, io)) {
    Some(Ok(())) => {}
    Some(Err(error)) => {
      warn!("{error}");
      info!(
        "{}",
        BenchLine::Refused {
          channel: channel.get(),
          reason: error
        }
      );
      return false;
    }
    None => return false
  }
  // Any key but the end of a line stops the move: a terminal that sends CR and LF leaves the LF waiting.
  let key = async {
    loop {
      let mut byte = [0_u8; 1];
      if matches!(rx.read(&mut byte).await, Ok(1)) && !matches!(byte[0], b'\r' | b'\n') {
        return;
      }
    }
  };
  let outcome = match select(MOVE_FINISHED.wait(), key).await {
    Either::First(outcome) => outcome,
    Either::Second(()) => {
      info!("key pressed: stopping");
      with_node(Node::stop);
      MOVE_FINISHED.wait().await
    }
  };
  if let Some(axis) = with_node(|node, _| node.axis(channel)).flatten() {
    info!(
      "{}",
      BenchLine::Moved {
        channel: channel.get(),
        end: BenchEnd::of(&outcome),
        pos_steps: axis.record.pos_steps,
        suspect: axis.record.suspect
      }
    );
  }
  matches!(outcome, MoveOutcome::Finished { end: MoveEnd::Done, .. })
}
