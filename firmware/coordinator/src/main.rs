//! Coordinator firmware for the ESP32-C6 (milestone M5): the interlock and moves over the bus.
//!
//! The decisions are in `coordinator-logic` (`Orchestrator`), tested on the host; this file wires them to the
//! hardware. Every pin is in `board.rs`, which is PROVISIONAL until the coordinator board exists.
//!
//! What this image does to hardware: it drives the relay output, which on the bench is the node's motor permit,
//! only as the state machine of reference 7.3 commands; a node moves a motor only after the relay's feedback
//! confirms the laser loop open and INHIBIT 0 has gone out. It reads the relay feedback, Status, and fire sense, and
//! drives nothing else: FootSW and measurement come with the sensor.
//!
//! Tasks:
//! - `control`: every millisecond, samples the inputs and hands them, the frames received, and the console's
//!   commands to the orchestrator, which owns the relay output. Feeds the watchdog.
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

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use align_core::config::Config;
use align_proto::{Envelope, Message, NodeId, PROTOCOL_VERSION};
use coordinator_logic::console::{self, Command};
use coordinator_logic::nodes::{Change, WATCHED};
use coordinator_logic::orchestrator::{CoordinatorIo, Inputs, Orchestrator, Report, Setup, WIRING};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as Queue;
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
use log::{error, info, warn};

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

type Raw = CriticalSectionRawMutex;

static FRAMES_OUT: Queue<Raw, Envelope, 16> = Queue::new();
static FRAMES_IN: Queue<Raw, Envelope, 16> = Queue::new();
static COMMANDS: Queue<Raw, Command, 4> = Queue::new();
/// The controller's error counters and bus-off state as the `can` task last read them, for `status`.
static TX_ERRORS: AtomicU8 = AtomicU8::new(0);
static RX_ERRORS: AtomicU8 = AtomicU8::new(0);
static BUS_OFF: AtomicBool = AtomicBool::new(false);

fn now_ms() -> u64 {
  Instant::now().as_millis()
}

/// The firmware side of [`CoordinatorIo`]: the transmit queue, the relay output, and the log.
struct Firmware<'a> {
  relay: &'a mut Output<'static>
}

impl CoordinatorIo for Firmware<'_> {
  fn send(&mut self, envelope: Envelope) {
    if FRAMES_OUT.try_send(envelope).is_err() {
      warn!("CAN transmit queue full: dropped {:?}", envelope.message);
    }
  }

  fn set_relay(&mut self, energized: bool) {
    self.relay.set_level(Level::from(energized));
  }

  fn report(&mut self, report: Report) {
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
      Report::MoveDropped { axis } => warn!("{axis:?}: the move was dropped before it was sent")
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

/// The inputs `control` samples, each read in its logical sense (see `board.rs` for the polarities).
struct Sensed {
  relay_feedback: Input<'static>,
  status: Input<'static>,
  fire: Input<'static>
}

impl Sensed {
  fn read(&self) -> Inputs {
    Inputs {
      loop_open: self.relay_feedback.is_high(),
      job_running: self.status.is_low(),
      firing: self.fire.is_low()
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

  // The relay output first, low: released before anything else runs.
  let pins = board_pins!(peripherals);
  let relay = Output::new(pins.relay, Level::Low, OutputConfig::default());
  let sensed = Sensed {
    relay_feedback: Input::new(pins.relay_feedback, InputConfig::default().with_pull(Pull::Down)),
    status: Input::new(pins.status, InputConfig::default().with_pull(Pull::Up)),
    fire: Input::new(pins.fire, InputConfig::default().with_pull(Pull::Up))
  };
  let orchestrator = Orchestrator::new(Setup {
    config: settings,
    fw_version: FW_VERSION,
    expected: EXPECTED,
    wiring: WIRING
  });

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
  // The watchdog resets the chip, and with it the relay output, if `control` stops feeding it: a panic spins with
  // interrupts masked, and a hang stops the loop. The nodes stop on their own when the heartbeat stops; the reset
  // also drops the motor permit. The node's timeout is used: nothing here measured a better one.
  let mut watchdog = TimerGroup::new(peripherals.TIMG1).wdt;
  watchdog.set_timeout(
    MwdtStage::Stage0,
    HalDuration::from_millis(u64::from(settings.node.watchdog_timeout_ms))
  );
  watchdog.enable();
  spawn!(control(orchestrator, relay, sensed, watchdog));
  spawn!(can(twai));
  spawn!(console_task(usb_rx));
  info!("coordinator ready; `help` on this console lists the commands");

  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}

#[embassy_executor::task]
async fn control(
  mut orchestrator: Orchestrator,
  mut relay: Output<'static>,
  sensed: Sensed,
  mut watchdog: Wdt<TIMG1<'static>>
) {
  let mut ticker = Ticker::every(Duration::from_millis(1));
  loop {
    watchdog.feed();
    let now = now_ms();
    let inputs = sensed.read();
    let mut io = Firmware { relay: &mut relay };
    while let Ok(envelope) = FRAMES_IN.try_receive() {
      orchestrator.on_frame(now, envelope, &mut io);
    }
    while let Ok(command) = COMMANDS.try_receive() {
      execute(now, command, inputs, &mut orchestrator, &mut io);
    }
    orchestrator.tick(now, inputs, &mut io);
    ticker.next().await;
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
    Command::Clear => orchestrator.clear_fault(now, io)
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
    "state {:?}; relay {}, motion {}",
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
