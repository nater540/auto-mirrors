//! Coordinator firmware for the ESP32-C6 (milestone M5), first step: the bus.
//!
//! The decisions are in `coordinator-logic`, tested on the host; this file wires them to the hardware.
//! Every pin is in `board.rs`, which is PROVISIONAL until the coordinator board exists.
//!
//! What this image does to hardware: it starts TWAI and sends its heartbeat with motion inhibited. It never sends
//! INHIBIT 0, MOVE_REL, or CONFIG, so no node moves a motor or changes a setting on its command. It drives no other
//! pin: the relay, FootSW, and the Ruida inputs arrive with the state machine (`align_core::coordinator`).
//!
//! Tasks:
//! - `clock`: ticks the link every 10 ms (the heartbeat and the inhibit level out, lost nodes found).
//! - `can`: receives and decodes frames for the link, and sends what it queues. Recovers from bus-off.
//! - `console_task`: the console over USB-Serial-JTAG; replies go to the log.
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

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use align_core::config::Config;
use align_proto::{Envelope, Message, NodeId, PROTOCOL_VERSION};
use coordinator_logic::console::{self, Command};
use coordinator_logic::link::{Beacon, Link};
use coordinator_logic::nodes::{Change, WATCHED};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as Queue;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use embedded_can::Frame as _;
use embedded_io_async::Read as _;
use esp_backtrace as _;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::twai::{BaudRate, EspTwaiFrame, Twai};
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx};
use log::{error, info, warn};

/// What this image does to motors and I/O: the text `just flash` shows before flashing.
const FLASH_NOTICE: &str = include_str!("../FLASH_NOTICE.txt");

// The app descriptor the ESP-IDF second-stage bootloader requires in every image.
esp_bootloader_esp_idf::esp_app_desc!();

/// Firmware version, reported in the heartbeat.
const FW_VERSION: u8 = 1;

/// 250 kbit/s (reference 5.3; an UNVERIFIED choice for a 2 to 3 m drag-chain bus), as the nodes.
const BAUD_RATE: BaudRate = BaudRate::B250K;

/// What this image says about itself: always inhibited. The heartbeat's state byte has no defined values until the
/// state machine runs here; 0 is the place of `align_core::coordinator::State::Inhibited`, its first state.
const BEACON: Beacon = Beacon {
  state: 0,
  inhibited: true
};

/// How often the link is ticked: a tenth of the heartbeat period, so a heartbeat is at most 10 ms late.
const TICK_MS: u64 = 10;

/// Longest console line; a longer one is discarded rather than run truncated.
const LINE_LEN: usize = 80;

type Raw = CriticalSectionRawMutex;

static LINK: Mutex<Raw, RefCell<Option<Link>>> = Mutex::new(RefCell::new(None));
static FRAMES_OUT: Queue<Raw, Envelope, 16> = Queue::new();
/// The controller's error counters and bus-off state as the `can` task last read them, for the console.
static TX_ERRORS: AtomicU8 = AtomicU8::new(0);
static RX_ERRORS: AtomicU8 = AtomicU8::new(0);
static BUS_OFF: AtomicBool = AtomicBool::new(false);

/// Runs `f` on the link. `None` before the link exists, or if it is already borrowed (a bug here: nothing called
/// from inside `f` reaches the link again).
fn with_link<R>(f: impl FnOnce(&mut Link) -> R) -> Option<R> {
  LINK.lock(|cell| cell.try_borrow_mut().ok()?.as_mut().map(f))
}

fn now_ms() -> u64 {
  Instant::now().as_millis()
}

/// Queues a frame for the `can` task.
fn send(envelope: Envelope) {
  if FRAMES_OUT.try_send(envelope).is_err() {
    warn!("CAN transmit queue full: dropped {:?}", envelope.message);
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
fn report(node: NodeId, change: Change) {
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

  let pins = board_pins!(peripherals);
  LINK.lock(|cell| cell.replace(Some(Link::new(&settings.node, FW_VERSION))));

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
  spawn!(clock());
  spawn!(can(twai));
  spawn!(console_task(usb_rx));
  info!("coordinator ready; `help` on this console lists the commands");

  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}

#[embassy_executor::task]
async fn clock() {
  let mut ticker = Ticker::every(Duration::from_millis(TICK_MS));
  loop {
    let now = now_ms();
    if let Some(lost) = with_link(|link| link.tick(now, BEACON, &mut send)) {
      for (node, change) in WATCHED.iter().zip(lost) {
        if let Some(change) = change {
          report(*node, change);
        }
      }
    }
    ticker.next().await;
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

/// A decoded frame: the link keeps heartbeats and STATUS, and the rest that matters is logged.
fn heard(envelope: Envelope) {
  let now = now_ms();
  if let Some(Some((node, change))) = with_link(|link| link.on_frame(now, envelope)) {
    report(node, change);
  }
  let id = envelope.node.get();
  match envelope.message {
    Message::Fault { channel, code, detail } => warn!(
      "node {id}: FAULT on channel {}, code {:#04x}, detail {detail}",
      channel.get(),
      code.0
    ),
    Message::MoveDone(done) => info!("node {id}: {done:?}"),
    _ => {}
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
          execute(text);
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

fn execute(text: &str) {
  match console::parse(text) {
    Err(error) => warn!("{error}"),
    Ok(Command::Help) => {
      for line in console::HELP.lines() {
        info!("{line}");
      }
    }
    Ok(Command::Status) => status()
  }
}

/// Prints the bus, and what each node last said.
fn status() {
  info!(
    "up {} ms, motion inhibited (this image never allows it); bus: transmit errors {}, receive errors {}{}",
    now_ms(),
    TX_ERRORS.load(Ordering::Acquire),
    RX_ERRORS.load(Ordering::Acquire),
    if BUS_OFF.load(Ordering::Acquire) {
      ", bus-off"
    } else {
      ""
    }
  );
  with_link(|link| {
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
  });
}
