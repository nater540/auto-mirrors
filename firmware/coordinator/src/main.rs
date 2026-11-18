//! Coordinator firmware for the ESP32-C6 (milestones M5 and M6): the interlock, moves and measurements over the bus,
//! the alignment procedure, and the bring-up measurements of backlash and hysteresis.
//!
//! The decisions are in `coordinator-logic` (`Orchestrator`), tested on the host; this file wires them to the
//! hardware. Every pin is in `board.rs`, which is PROVISIONAL until the coordinator board exists.
//!
//! What this image does to hardware: it drives the relay output, which on the bench is the node's motor permit,
//! only as the state machine of reference 7.3 commands; a node moves a motor only after the relay's feedback
//! confirms the laser loop open and INHIBIT 0 has gone out. It pulses FootSW, which starts the Ruida's job, only for
//! a measurement in Idle with the relay released. It reads the relay feedback, Status, and fire sense. The console's
//! `calibrate`, `align`, `poke`, `backlash`, and `hysteresis` move and measure through the same orchestrator, one
//! request at a time.
//!
//! Tasks:
//! - `control`: every millisecond, samples the inputs and hands them, the frames received, and the console's
//!   commands to the orchestrator, which owns the relay and FootSW outputs. Feeds the watchdog. It does not log what
//!   the orchestrator reports: a burst of lines over USB could hold it for milliseconds and miss a dot.
//! - `fire_sense`: keeps the fire input's level and latches each of its edges from the pin's interrupt, so that a
//!   dot shorter than a millisecond, or one during a late loop, still reaches the next sample.
//! - `logger`: logs what the orchestrator reports, from a queue.
//! - `procedure`: runs the aligner (`align_core::aligner::Aligner`) and the bring-up measurements. Its moves and
//!   measurements go to `control` as requests (`coordinator_logic::requests`) and come back as completions; a fault of
//!   the procedure faults the coordinator, cleared by hand. The bring-up logs a `BU1` line per measurement, per
//!   trial, and for its report (`align_core::bringup`), for the owner to paste back. After a procedure, with no move
//!   running, it writes a new calibration and a successful alignment to flash (`coordinator_logic::stored`): each
//!   flash operation masks interrupts, so nothing else may be in flight, and the next boot starts from them.
//! - `can`: receives and decodes frames for `control`, and sends what it queues. Recovers from bus-off.
//! - `console_task`: reads the console over USB-Serial-JTAG and queues its commands; replies go to the log.
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

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use align_core::aligner::{AlignFault, AlignReport, Aligner};
use align_core::axis::{Axis, PerAxis};
use align_core::bringup::{Play, SAMPLE_COLUMNS};
use align_core::calibration::Calibration;
use align_core::config::Config;
use align_core::measurement::Shots;
use align_core::rig::{Actuator, BeamSensor, MoveFault, SensorFault};
use align_proto::{Envelope, Message, NodeId, PROTOCOL_VERSION};
use coordinator_logic::console::{self, Command};
use coordinator_logic::nodes::{Change, WATCHED};
use coordinator_logic::orchestrator::{CoordinatorIo, Inputs, Orchestrator, Report, Setup, WIRING};
use coordinator_logic::requests::{Completion, Reported, Request, Requests, reported, starting_axes};
use coordinator_logic::stored::{self, CALIBRATION_KEY, LAST_GOOD_KEY, LastGood};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as Queue;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
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
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage, SerializationError, Value};

/// What this image does to motors and I/O: the text `just flash` shows before flashing.
const FLASH_NOTICE: &str = include_str!("../FLASH_NOTICE.txt");

// The app descriptor the ESP-IDF second-stage bootloader requires in every image.
esp_bootloader_esp_idf::esp_app_desc!();

/// Firmware version, reported in the heartbeat.
const FW_VERSION: u8 = 2;

/// 250 kbit/s (reference 5.3; an UNVERIFIED choice for a 2 to 3 m drag-chain bus), as the nodes.
const BAUD_RATE: BaudRate = BaudRate::B250K;

/// PROVISIONAL: the nodes the self-test needs. The bench has the mirror 1 node only; the machine needs both mirror
/// nodes, and the sensor once measurement runs here.
const EXPECTED: &[NodeId] = &[NodeId::MIRROR_1];

/// Longest console line; a longer one is discarded rather than run truncated.
const LINE_LEN: usize = 80;

/// How long one of the procedure's requests may wait for the coordinator to take it.
const REQUEST_WAIT_MS: u32 = 60_000;

/// Reports one pass of the control loop can hold before they are drained; the rest are counted as dropped.
const REPORTS_PER_PASS: usize = 16;

type Raw = CriticalSectionRawMutex;

/// Room for a burst: a node's six CONFIG frames, the heartbeat, the inhibit level, ESTOP, and a move.
static FRAMES_OUT: Queue<Raw, Envelope, 32> = Queue::new();
static FRAMES_IN: Queue<Raw, Envelope, 16> = Queue::new();
static COMMANDS: Queue<Raw, Command, 4> = Queue::new();
/// The controller's error counters and bus-off state as the `can` task last read them, for `status`.
static TX_ERRORS: AtomicU8 = AtomicU8::new(0);
static RX_ERRORS: AtomicU8 = AtomicU8::new(0);
static BUS_OFF: AtomicBool = AtomicBool::new(false);
/// What the orchestrator reported, for `logger`; reports that found it full are counted, not waited for.
static REPORTS: Queue<Raw, Report, 32> = Queue::new();
static REPORTS_DROPPED: AtomicU32 = AtomicU32::new(0);
/// The fire input as `fire_sense` keeps it: whether it is active now, and whether it changed since `control`
/// last looked.
static FIRE_ACTIVE: AtomicBool = AtomicBool::new(false);
static FIRE_EDGE: AtomicBool = AtomicBool::new(false);
/// The procedure's request to `control`, and its completion back: one at a time.
static REQUESTS: Queue<Raw, Request, 1> = Queue::new();
static COMPLETION: Signal<Raw, Completion> = Signal::new();
/// A procedure to run, with what the nodes last reported of each axis, from `control`.
static PROCEDURES: Queue<Raw, Procedure, 1> = Queue::new();
/// The procedure ended in a fault: `control` faults the coordinator.
static PROCEDURE_FAULT: Signal<Raw, ()> = Signal::new();
static PROCEDURE_RUNNING: AtomicBool = AtomicBool::new(false);

type Flash = BlockingAsync<FlashStorage<'static>>;
type Records = MapStorage<u8, Flash, Cache<Uncached, Uncached, Uncached, u8>>;

/// Room for one key and the larger record (the calibration's 89 bytes), rounded up to the flash word, with room
/// to spare.
const STORAGE_BUFFER_LEN: usize = 128;

/// The calibration as sequential-storage stores it.
struct StoredCalibration(Calibration);

impl Value<'_> for StoredCalibration {
  fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
    let bytes = stored::calibration_to_bytes(&self.0);
    buffer
      .get_mut(..bytes.len())
      .ok_or(SerializationError::BufferTooSmall)?
      .copy_from_slice(&bytes);
    Ok(bytes.len())
  }

  fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
    let calibration = stored::calibration_from_bytes(buffer).map_err(|_| SerializationError::InvalidData)?;
    Ok((Self(calibration), buffer.len()))
  }
}

/// The last good alignment as sequential-storage stores it.
struct StoredLastGood(LastGood);

impl Value<'_> for StoredLastGood {
  fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
    let bytes = self.0.to_bytes();
    buffer
      .get_mut(..bytes.len())
      .ok_or(SerializationError::BufferTooSmall)?
      .copy_from_slice(&bytes);
    Ok(bytes.len())
  }

  fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
    let last_good = LastGood::from_bytes(buffer).map_err(|_| SerializationError::InvalidData)?;
    Ok((Self(last_good), buffer.len()))
  }
}

/// What the procedure is asked to do.
#[derive(Clone, Copy, Debug)]
enum ProcedureKind {
  Calibrate,
  Align,
  Poke(Axis),
  Backlash(Axis),
  Hysteresis(Axis)
}

/// A procedure, and the nodes' counts it starts from.
#[derive(Clone, Copy, Debug)]
struct Procedure {
  kind: ProcedureKind,
  reported: PerAxis<Option<Reported>>
}

fn now_ms() -> u64 {
  Instant::now().as_millis()
}

/// The outputs that act on the machine, both low (inactive) at boot.
struct Outputs {
  relay: Output<'static>,
  footsw: Output<'static>
}

/// The reports of one pass of the control loop, drained at its end into the log and the requests.
struct PassReports {
  reports: [Option<Report>; REPORTS_PER_PASS],
  len: usize
}

impl PassReports {
  const fn new() -> Self {
    Self {
      reports: [None; REPORTS_PER_PASS],
      len: 0
    }
  }

  fn push(&mut self, report: Report) {
    match self.reports.get_mut(self.len) {
      Some(slot) => {
        *slot = Some(report);
        self.len += 1;
      }
      None => {
        REPORTS_DROPPED.fetch_add(1, Ordering::AcqRel);
      }
    }
  }

  /// Hands every report to `each`, in order, and empties the pass.
  fn drain(&mut self, mut each: impl FnMut(Report)) {
    for slot in self.reports.iter_mut().take(self.len) {
      if let Some(report) = slot.take() {
        each(report);
      }
    }
    self.len = 0;
  }
}

/// The firmware side of [`CoordinatorIo`]: the transmit queue, the outputs, and this pass's reports.
struct Firmware<'a> {
  outputs: &'a mut Outputs,
  reports: &'a mut PassReports
}

impl CoordinatorIo for Firmware<'_> {
  fn send(&mut self, envelope: Envelope) {
    if FRAMES_OUT.try_send(envelope).is_err() {
      warn!("CAN transmit queue full: dropped {:?}", envelope.message);
    }
  }

  fn set_relay(&mut self, energized: bool) {
    self.outputs.relay.set_level(Level::from(energized));
  }

  fn set_footsw(&mut self, active: bool) {
    self.outputs.footsw.set_level(Level::from(active));
  }

  fn report(&mut self, report: Report) {
    self.reports.push(report);
  }
}

/// Logs one report of the orchestrator.
fn log_report(report: Report) {
  match report {
    Report::State(state) => info!("state {state:?}"),
    Report::Outcome(outcome) => info!("outcome {outcome:?}"),
    Report::Node { node, change } => report_node(node, change),
    Report::Configured { node } => info!(
      "node {} ({}): soft limits and direction signs sent",
      node.get(),
      name(node)
    ),
    Report::MoveSent { axis, steps } => info!("{axis:?}: {steps} steps sent"),
    Report::MoveEnded { axis, result } => match result {
      Ok(pos_steps) => info!("{axis:?}: done, at {pos_steps} steps"),
      Err(fault) => warn!("{axis:?}: {fault}")
    },
    Report::MoveDropped { axis } => warn!("{axis:?}: the move was dropped before it was sent"),
    Report::MeasurementEnded { result } => match result {
      Ok(shots) => {
        for (point, reading) in [("A", shots.a), ("B", shots.b), ("C", shots.c)] {
          info!(
            "measured {point}: h {:.4}, v {:.4}, sum {:.3}",
            reading.h_norm, reading.v_norm, reading.sum
          );
        }
      }
      Err(fault) => warn!("measurement: {fault}")
    }
  }
}

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

/// What the console calls each node.
fn name(node: NodeId) -> &'static str {
  match node {
    NodeId::MIRROR_1 => "mirror 1",
    NodeId::MIRROR_2 => "mirror 2",
    NodeId::SENSOR => "sensor",
    _ => "unknown"
  }
}

/// Logs a change in what is known about a node.
fn report_node(node: NodeId, change: Change) {
  let id = node.get();
  let name = name(node);
  match change {
    Change::Appeared => info!("node {id} ({name}): present"),
    Change::Restarted => warn!("node {id} ({name}): restarted"),
    Change::Lost => warn!("node {id} ({name}): lost, no heartbeat within the timeout"),
    Change::ProtocolMismatch { version } => {
      warn!("node {id} ({name}): protocol version {version}, not {PROTOCOL_VERSION}; not counted as present");
    }
  }
}

/// The inputs `control` samples, each read in its logical sense (see `board.rs` for the polarities). Fire sense is
/// read through `fire_sense`.
struct Sensed {
  relay_feedback: Input<'static>,
  status: Input<'static>
}

impl Sensed {
  fn read(&self) -> Inputs {
    // An edge since the last sample counts as firing for this one, however short the dot was.
    let edge = FIRE_EDGE.swap(false, Ordering::AcqRel);
    Inputs {
      loop_open: self.relay_feedback.is_high(),
      job_running: self.status.is_low(),
      firing: FIRE_ACTIVE.load(Ordering::Acquire) || edge
    }
  }
}

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

  // The outputs first, low: the relay released and FootSW inactive before anything else runs.
  let pins = board_pins!(peripherals);
  let outputs = Outputs {
    relay: Output::new(pins.relay, Level::Low, OutputConfig::default()),
    footsw: Output::new(pins.footsw, Level::Low, OutputConfig::default())
  };
  let sensed = Sensed {
    relay_feedback: Input::new(pins.relay_feedback, InputConfig::default().with_pull(Pull::Down)),
    status: Input::new(pins.status, InputConfig::default().with_pull(Pull::Up))
  };
  let fire = Input::new(pins.fire, InputConfig::default().with_pull(Pull::Up));
  let orchestrator = Orchestrator::new(Setup {
    config: settings,
    fw_version: FW_VERSION,
    expected: EXPECTED,
    wiring: WIRING
  });

  // Before the watchdog starts: erasing unreadable records may take longer than its timeout. Without flash the
  // coordinator still runs, and keeps its calibration until reset.
  let mut records = open_records(FlashStorage::new(peripherals.FLASH));
  let calibration = if let Some(records) = records.as_mut() {
    restore(records).await
  } else {
    warn!("no usable flash partition for records: the calibration lasts until reset");
    None
  };

  let twai = board::twai(peripherals.TWAI0, pins.twai_tx, pins.twai_rx, BAUD_RATE)
    .into_async()
    .start();
  let (usb_rx, _usb_tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async().split();

  // Each task is spawned once into a pool of one, so creating its token cannot fail; if it ever did, the task would
  // be missing (no heartbeat, so every node stays inhibited), which the log names.
  macro_rules! spawn {
    ($task:expr) => {
      match $task {
        Ok(token) => spawner.spawn(token),
        Err(error) => error!("spawning `{}` failed: {error:?}", stringify!($task))
      }
    };
  }
  // The watchdog resets the chip, and with it both outputs, if `control` stops feeding it: a panic spins with
  // interrupts masked, and a hang stops the loop. The nodes stop on their own when the heartbeat stops; the reset
  // also drops the motor permit and ends a FootSW pulse. The node's timeout is used: nothing measured a better one.
  let mut watchdog = TimerGroup::new(peripherals.TIMG1).wdt;
  watchdog.set_timeout(
    MwdtStage::Stage0,
    HalDuration::from_millis(u64::from(settings.node.watchdog_timeout_ms))
  );
  watchdog.enable();
  spawn!(fire_sense(fire));
  spawn!(logger());
  spawn!(procedure(records, calibration));
  spawn!(control(orchestrator, outputs, sensed, watchdog));
  spawn!(can(twai));
  spawn!(console_task(usb_rx));
  info!("coordinator ready; `help` on this console lists the commands");

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
  info!("records in flash {:#x}..{:#x}", range.start, range.end);
  let config = MapConfig::try_new(range).ok()?;
  Some(MapStorage::new(
    BlockingAsync::new(flash),
    config,
    Cache::new_uncached()
  ))
}

/// Reads the calibration, to start from, and the last good alignment, to log. A record that does not read (another
/// format, a value out of range) is ignored and replaced by the next one written; storage that does not read at all
/// (for example, ESP-IDF NVS data left by an earlier image) is erased.
async fn restore(records: &mut Records) -> Option<Calibration> {
  let mut corrupt = false;
  let mut buffer = [0_u8; STORAGE_BUFFER_LEN];
  let calibration = match records
    .fetch_item::<StoredCalibration>(&mut buffer, &CALIBRATION_KEY)
    .await
  {
    Ok(found) => found.map(|StoredCalibration(calibration)| calibration),
    Err(sequential_storage::Error::SerializationError(_)) => {
      warn!("the stored calibration does not read: the next alignment calibrates");
      None
    }
    Err(error) => {
      warn!("reading the stored calibration failed: {error:?}");
      corrupt = true;
      None
    }
  };
  let mut buffer = [0_u8; STORAGE_BUFFER_LEN];
  let last_good = match records.fetch_item::<StoredLastGood>(&mut buffer, &LAST_GOOD_KEY).await {
    Ok(found) => found.map(|StoredLastGood(last_good)| last_good),
    Err(sequential_storage::Error::SerializationError(_)) => {
      warn!("the stored last good alignment does not read");
      None
    }
    Err(error) => {
      warn!("reading the stored last good alignment failed: {error:?}");
      corrupt = true;
      None
    }
  };
  if corrupt {
    warn!("erasing the record partition");
    if let Err(error) = records.erase_all().await {
      error!("erasing the record partition failed: {error:?}");
    }
    return None;
  }
  match &calibration {
    Some(calibration) => info!(
      "calibration restored from flash: condition number {:.2}, measured at {:?}",
      calibration.cond, calibration.q0_steps
    ),
    None => info!("no stored calibration: the first alignment calibrates")
  }
  if let Some(last_good) = &last_good {
    info!(
      "last good alignment: screws at {:?}, residual {:.4}",
      last_good.pos_steps, last_good.residual_norm
    );
  }
  calibration
}

/// Writes what a procedure changed: a new calibration, and the screws' positions after an alignment that converged.
/// Called only between procedures, with no move running.
async fn persist(records: &mut Records, calibration: Option<Calibration>, aligned: Option<LastGood>) {
  if let Some(calibration) = calibration {
    let mut buffer = [0_u8; STORAGE_BUFFER_LEN];
    match records
      .store_item(&mut buffer, &CALIBRATION_KEY, &StoredCalibration(calibration))
      .await
    {
      Ok(()) => info!("calibration written to flash"),
      Err(error) => error!("writing the calibration failed: {error:?}")
    }
  }
  if let Some(last_good) = aligned {
    let mut buffer = [0_u8; STORAGE_BUFFER_LEN];
    match records
      .store_item(&mut buffer, &LAST_GOOD_KEY, &StoredLastGood(last_good))
      .await
    {
      Ok(()) => info!("alignment written to flash: screws at {:?}", last_good.pos_steps),
      Err(error) => error!("writing the alignment failed: {error:?}")
    }
  }
}

#[embassy_executor::task]
async fn control(
  mut orchestrator: Orchestrator,
  mut outputs: Outputs,
  sensed: Sensed,
  mut watchdog: Wdt<TIMG1<'static>>
) {
  let mut ticker = Ticker::every(Duration::from_millis(1));
  let mut bus_was_off = false;
  let mut requests = Requests::new(Config::default().motion.settle_ms, REQUEST_WAIT_MS);
  let mut reports = PassReports::new();
  loop {
    watchdog.feed();
    let now = now_ms();
    let inputs = sensed.read();
    let started = {
      let mut io = Firmware {
        outputs: &mut outputs,
        reports: &mut reports
      };
      // Reference 7.3: bus-off is a fault, cleared by hand, even though the controller recovers on its own.
      let bus_off = BUS_OFF.load(Ordering::Acquire);
      if bus_off && !bus_was_off {
        orchestrator.fault(now, &mut io);
      }
      bus_was_off = bus_off;
      if PROCEDURE_FAULT.try_take().is_some() {
        orchestrator.fault(now, &mut io);
      }
      while let Ok(envelope) = FRAMES_IN.try_receive() {
        orchestrator.on_frame(now, envelope, &mut io);
      }
      while let Ok(command) = COMMANDS.try_receive() {
        execute(now, command, inputs, &mut orchestrator, &mut io);
      }
      if !requests.busy()
        && let Ok(request) = REQUESTS.try_receive()
      {
        // Not busy, so it is taken.
        let _ = requests.submit(now, request);
      }
      orchestrator.tick(now, inputs, &mut io);
      requests.poll(now, &mut orchestrator, &mut io)
    };
    if let Some(completion) = started {
      COMPLETION.signal(completion);
    }
    reports.drain(|report| {
      if let Some(completion) = requests.on_report(now, &report) {
        COMPLETION.signal(completion);
      }
      if REPORTS.try_send(report).is_err() {
        REPORTS_DROPPED.fetch_add(1, Ordering::AcqRel);
      }
    });
    ticker.next().await;
  }
}

/// The aligner's actuator and sensor: each move and measurement is a request to `control`, answered by its
/// completion.
#[derive(Clone, Copy)]
struct BusRig;

/// Sends one request to `control` and waits for its completion.
async fn ask(request: Request) -> Completion {
  COMPLETION.reset();
  REQUESTS.send(request).await;
  COMPLETION.wait().await
}

impl Actuator for BusRig {
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    match ask(Request::Move {
      axis,
      steps: delta_steps
    })
    .await
    {
      Completion::Move(result) => result,
      Completion::Measure(_) => Err(MoveFault::NoAnswer)
    }
  }
}

impl BeamSensor for BusRig {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    match ask(Request::Measure).await {
      Completion::Measure(result) => result,
      Completion::Move(_) => Err(SensorFault::NoReading)
    }
  }
}

#[embassy_executor::task]
async fn procedure(mut records: Option<Records>, restored: Option<Calibration>) {
  // The configuration `main` uses, built here: a task takes no borrowed arguments.
  let config = Config::default();
  // The sensor reports sums relative to its reference, so the aligner's reference sum is 1. The aligner keeps its
  // calibration and its axes between procedures.
  let mut aligner = match Aligner::new(BusRig, BusRig, config, 1.0) {
    Ok(aligner) => aligner,
    Err(fault) => {
      error!("the alignment procedure refuses its configuration: {fault}");
      return;
    }
  };
  // The aligner checks a restored calibration as it does any: its conditioning, and its direction signs against
  // the configuration. One that fails is measured again.
  if let Some(calibration) = restored {
    aligner = aligner.with_calibration(calibration);
  }
  loop {
    let Procedure { kind, reported } = PROCEDURES.receive().await;
    PROCEDURE_RUNNING.store(true, Ordering::Release);
    let axes = starting_axes(aligner.axes(), &reported);
    aligner = aligner.with_axes(axes);
    let calibration_before = aligner.calibration().copied();
    let aligned = match run_procedure(&mut aligner, kind, config.motion.poke_steps).await {
      Ok(report) => report.map(|report| LastGood::of(aligner.axes(), report.residual_norm)),
      Err(fault) => {
        // Any fault of the procedure stops the coordinator until cleared: someone looks before the next run.
        warn!("{kind:?}: {fault}; the coordinator is faulted until `clear`");
        PROCEDURE_FAULT.signal(());
        None
      }
    };
    // A calibration the procedure measured is kept whether or not the procedure then succeeded: the aligner
    // accepted it. Written while `PROCEDURE_RUNNING` holds off the next procedure, with every move finished.
    let calibration = aligner
      .calibration()
      .copied()
      .filter(|now| Some(*now) != calibration_before);
    if let Some(records) = records.as_mut() {
      persist(records, calibration, aligned).await;
    }
    PROCEDURE_RUNNING.store(false, Ordering::Release);
  }
}

/// Runs one procedure; the report of an alignment that converged.
async fn run_procedure(
  aligner: &mut Aligner<BusRig, BusRig>,
  kind: ProcedureKind,
  poke_steps: i32
) -> Result<Option<AlignReport>, AlignFault> {
  match kind {
    ProcedureKind::Calibrate => {
      let calibration = aligner.calibrate().await?;
      info!("calibrated: condition number {:.2}", calibration.cond);
    }
    ProcedureKind::Align => {
      let report = aligner.align().await?;
      info!(
        "aligned: {} steps, {} recalibrations, residual {:.4}, condition number {:.2}, head offset ({:+.3}, {:+.3}){}",
        report.iterations,
        report.recalibrations,
        report.residual_norm,
        report.cond,
        report.head_offset.h_norm,
        report.head_offset.v_norm,
        if report.needs_mechanical_centering {
          ", mechanical centering needed"
        } else {
          ""
        }
      );
      return Ok(Some(report));
    }
    ProcedureKind::Poke(axis) => {
      let mut sensor = BusRig;
      let before = sensor.measure().await.map_err(AlignFault::Sensor)?;
      aligner.approach(axis, poke_steps).await?;
      let after = sensor.measure().await.map_err(AlignFault::Sensor)?;
      aligner.approach(axis, poke_steps.saturating_neg()).await?;
      #[expect(clippy::cast_precision_loss, reason = "a poke is a few thousand steps at most")]
      let column = (after.measurement() - before.measurement()) / poke_steps as f32;
      info!(
        "poke {axis:?} by {poke_steps} steps: column ({:+.5}, {:+.5}, {:+.5}, {:+.5}) per step",
        column.x, column.y, column.z, column.w
      );
    }
    ProcedureKind::Backlash(axis) => {
      info!("{SAMPLE_COLUMNS}");
      let report = aligner.measure_backlash(axis, &mut |entry| info!("{entry}")).await?;
      info!("{report}");
      match report.play {
        Play::Bracket {
          above_steps,
          up_to_steps
        } => info!(
          "backlash {}: play more than {above_steps} and at most {up_to_steps} steps, to {:.1} steps; the line \
           above gives addendum A.5's overshoot",
          axis.name(),
          report.resolution_steps()
        ),
        Play::Beyond { steps } => warn!(
          "backlash {}: no trial moved the beam, so the play is more than {steps} steps",
          axis.name()
        )
      }
    }
    ProcedureKind::Hysteresis(axis) => {
      info!("{SAMPLE_COLUMNS}");
      let report = aligner.measure_hysteresis(axis, &mut |entry| info!("{entry}")).await?;
      info!("{report}");
      match report.width_steps {
        Some(width) => info!(
          "hysteresis {}: the loop is {width:.1} steps wide over {} steps either way",
          axis.name(),
          report.amplitude_steps
        ),
        None => warn!(
          "hysteresis {}: the loop never opened; the play may be more than the cycle",
          axis.name()
        )
      }
    }
  }
  Ok(None)
}

#[embassy_executor::task]
async fn fire_sense(mut fire: Input<'static>) {
  loop {
    FIRE_ACTIVE.store(fire.is_low(), Ordering::Release);
    fire.wait_for_any_edge().await;
    FIRE_EDGE.store(true, Ordering::Release);
  }
}

#[embassy_executor::task]
async fn logger() {
  loop {
    let report = REPORTS.receive().await;
    let dropped = REPORTS_DROPPED.swap(0, Ordering::AcqRel);
    if dropped > 0 {
      warn!("{dropped} reports were dropped: the log fell behind");
    }
    log_report(report);
  }
}

fn execute(now: u64, command: Command, inputs: Inputs, orchestrator: &mut Orchestrator, io: &mut Firmware<'_>) {
  match command {
    Command::Help => {
      for line in console::HELP.lines() {
        info!("{line}");
      }
    }
    Command::Status => status(inputs, orchestrator),
    Command::Jog { axis, steps } => {
      if let Err(error) = orchestrator.request_move(now, axis, steps, io) {
        warn!("{axis:?}: {error}");
      }
    }
    Command::Stop => orchestrator.stop(io),
    Command::Measure => {
      if let Err(error) = orchestrator.request_measurement(now, io) {
        warn!("measure: {error}");
      }
    }
    Command::Calibrate => start_procedure(ProcedureKind::Calibrate, orchestrator),
    Command::Align => start_procedure(ProcedureKind::Align, orchestrator),
    Command::Poke { axis } => start_procedure(ProcedureKind::Poke(axis), orchestrator),
    Command::Backlash { axis } => start_procedure(ProcedureKind::Backlash(axis), orchestrator),
    Command::Hysteresis { axis } => start_procedure(ProcedureKind::Hysteresis(axis), orchestrator),
    Command::Clear => orchestrator.clear_fault(now, io)
  }
}

/// Hands a procedure to the `procedure` task with what the nodes last reported of each axis.
fn start_procedure(kind: ProcedureKind, orchestrator: &Orchestrator) {
  if PROCEDURE_RUNNING.load(Ordering::Acquire) {
    warn!("{kind:?}: a procedure is running");
    return;
  }
  let procedure = Procedure {
    kind,
    reported: reported(orchestrator.link(), &WIRING)
  };
  if PROCEDURES.try_send(procedure).is_err() {
    warn!("{kind:?}: a procedure is already waiting to start");
  }
}

#[embassy_executor::task]
async fn can(mut twai: Twai<'static, Async>) {
  let mut rejected = RateLimited::new();
  // Alone on the bus (no node powered), every frame fails: twenty a second.
  let mut unsent = RateLimited::new();
  loop {
    TX_ERRORS.store(twai.transmit_error_count(), Ordering::Release);
    RX_ERRORS.store(twai.receive_error_count(), Ordering::Release);
    BUS_OFF.store(twai.is_bus_off(), Ordering::Release);
    if twai.is_bus_off() {
      // Reference 5.3: wait 100 ms, recover once, and wait for the recovery to finish rather than starting it again.
      // The nodes stop on their own meanwhile: no heartbeat reaches them.
      warn!("CAN bus-off: recovering in 100 ms");
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
            Ok(envelope) => heard(envelope),
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

/// A decoded frame: queued for `control`, and a node's FAULT logged, since the orchestrator acts only on those
/// that end a move.
fn heard(envelope: Envelope) {
  if let Message::Fault { channel, code, detail } = envelope.message {
    warn!(
      "node {}: FAULT on channel {}, code {:#04x}, detail {detail}",
      envelope.node.get(),
      channel.get(),
      code.0
    );
  }
  if FRAMES_IN.try_send(envelope).is_err() {
    warn!("CAN receive queue full: dropped {:?}", envelope.message);
  }
}

#[embassy_executor::task]
async fn console_task(mut rx: UsbSerialJtagRx<'static, Async>) {
  let mut line = [0_u8; LINE_LEN];
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
          match console::parse(text) {
            Ok(command) => {
              if COMMANDS.try_send(command).is_err() {
                warn!("busy: command dropped");
              }
            }
            Err(error) => warn!("{error}")
          }
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

/// Prints the state, the inputs, the bus, and what each node last said.
fn status(inputs: Inputs, orchestrator: &Orchestrator) {
  info!(
    "state {:?}; relay {}, motion {}; procedure {}",
    orchestrator.state(),
    if orchestrator.relay_energized() {
      "energized"
    } else {
      "released"
    },
    if orchestrator.motion_allowed() {
      "allowed"
    } else {
      "inhibited"
    },
    if PROCEDURE_RUNNING.load(Ordering::Acquire) {
      "running"
    } else {
      "idle"
    }
  );
  info!(
    "inputs: laser loop {}, job {}, fire {}; up {} ms",
    if inputs.loop_open { "open" } else { "closed" },
    if inputs.job_running { "running" } else { "idle" },
    if inputs.firing { "active" } else { "idle" },
    now_ms()
  );
  info!(
    "bus: transmit errors {}, receive errors {}{}",
    TX_ERRORS.load(Ordering::Acquire),
    RX_ERRORS.load(Ordering::Acquire),
    if BUS_OFF.load(Ordering::Acquire) {
      ", bus-off"
    } else {
      ""
    }
  );
  let link = orchestrator.link();
  for node in WATCHED {
    let id = node.get();
    let name = name(node);
    let Some(heard) = link.heard(node) else {
      continue;
    };
    let presence = if link.present(node) { "present" } else { "not present" };
    match (heard.state, heard.fw_version, heard.uptime_ms) {
      (Some(state), Some(fw_version), Some(uptime_ms)) => info!(
        "node {id} ({name}): {presence}; last heartbeat: state {state}, firmware {fw_version}, uptime {uptime_ms} ms"
      ),
      _ => info!("node {id} ({name}): never heard")
    }
    for status in heard.status.iter().flatten() {
      info!(
        "  channel {}: position {} steps, flags {:?}",
        status.channel.get(),
        status.pos_steps,
        status.flags
      );
    }
  }
}
