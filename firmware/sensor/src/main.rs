//! Sensor node firmware for the ESP32-C6 (`docs/SENSOR_PLAN.md`, revision 4).
//!
//! The decisions are in `sensor-logic`, tested on the host; this file wires them to the hardware.
//! Five buttons stand in for the detector, which is not chosen yet; every pin is in `board.rs`.
//!
//! Tasks:
//! - `buttons`: samples the five inputs every 5 ms, debounces them, and reports the held set and each chord.
//! - `clock`: runs the node's clock every 10 ms: readings that are due, and the heartbeat.
//! - `can`: receives and decodes frames for the node, and sends what it queues. Recovers from bus-off.
//! - `console`: the bench console over USB-Serial-JTAG; replies go to the log.
//!
//! This image drives no output pin and writes nothing to flash. `FLASH_NOTICE.txt` says so; the image logs it at
//! boot, and `just flash` prints it before flashing.
#![no_std]
#![no_main]
#![deny(
  clippy::mem_forget,
  reason = "mem::forget is not safe with esp-hal types that hold a buffer for the length of a transfer"
)]
#![deny(clippy::large_stack_frames)]

mod board;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};

use align_core::config::Config;
use align_proto::{Envelope, Message, NodeId, Shot};
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
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::twai::{BaudRate, EspTwaiFrame, Twai};
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx};
use log::{error, info, warn};
use sensor_logic::console::{self, Command};
use sensor_logic::node::sum_q_from_percent;
use sensor_logic::{ButtonBench, ButtonSet, Chords, Debouncer, SensorIo, SensorNode, SensorSetup};
use static_cell::StaticCell;

// The app descriptor the ESP-IDF second-stage bootloader requires in every image.
esp_bootloader_esp_idf::esp_app_desc!();

/// What this image does to motors and I/O: the text `just flash` shows before flashing.
const FLASH_NOTICE: &str = include_str!("../FLASH_NOTICE.txt");

/// Firmware version, reported in the heartbeat.
const FW_VERSION: u8 = 1;

/// 250 kbit/s (reference 5.3; an UNVERIFIED choice for a 2 to 3 m drag-chain bus).
const BAUD_RATE: BaudRate = BaudRate::B250K;

type Raw = CriticalSectionRawMutex;

static NODE: Mutex<Raw, RefCell<Option<SensorNode>>> = Mutex::new(RefCell::new(None));
static FRAMES_OUT: Queue<Raw, Envelope, 16> = Queue::new();
/// Buttons the console holds, added to the physical ones.
static CONSOLE_HELD: AtomicU8 = AtomicU8::new(0);
/// Sequence numbers of SHOT frames the console injects.
static CONSOLE_SEQ: AtomicU8 = AtomicU8::new(0);

/// The firmware side of [`SensorIo`]: the transmit queue.
struct Outbox;

impl SensorIo for Outbox {
  fn send(&mut self, envelope: Envelope) {
    if FRAMES_OUT.try_send(envelope).is_err() {
      warn!("CAN transmit queue full: dropped {:?}", envelope.message);
    }
  }
}

/// Runs `f` on the node. `None` before the node exists, or if it is already borrowed:
/// that would be a bug here, because [`Outbox`] never calls back into the node.
fn with_node<R>(f: impl FnOnce(&mut SensorNode, &mut Outbox) -> R) -> Option<R> {
  NODE.lock(|cell| cell.try_borrow_mut().ok()?.as_mut().map(|node| f(node, &mut Outbox)))
}

fn now_ms() -> u64 {
  Instant::now().as_millis()
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
  esp_println::logger::init_logger_from_env();

  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

  for line in FLASH_NOTICE.lines() {
    info!("{line}");
  }

  let pins = board_pins!(peripherals);
  let pulled_down = InputConfig::default().with_pull(Pull::Down);
  let buttons = pins.buttons.map(|pin| Input::new(pin, pulled_down));

  let settings = Config::default();
  let node = SensorNode::new(SensorSetup {
    fw_version: FW_VERSION,
    timing: settings.node,
    sensor_timing: settings.sensor_timing,
    bench: ButtonBench::STARTING
  });
  NODE.lock(|cell| cell.replace(Some(node)));

  let twai = board::twai(peripherals.TWAI0, pins.twai_tx, BAUD_RATE)
    .into_async()
    .start();
  let (usb_rx, _usb_tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async().split();

  // Each task is spawned once into a pool of one, so creating its token cannot fail.
  // If it ever did, the log names the missing task.
  macro_rules! spawn {
    ($task:expr) => {
      match $task {
        Ok(token) => spawner.spawn(token),
        Err(error) => error!("spawning `{}` failed: {error:?}", stringify!($task))
      }
    };
  }
  spawn!(buttons_task(buttons));
  spawn!(clock());
  spawn!(can(twai));
  spawn!(console_task(usb_rx));
  info!(
    "sensor node {} ready; `help` on this console lists the bench commands",
    NodeId::SENSOR.get()
  );

  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}

#[embassy_executor::task]
async fn buttons_task(buttons: [Input<'static>; 5]) {
  let bench = ButtonBench::STARTING;
  let mut debouncer = Debouncer::default();
  let mut chords = Chords::default();
  let mut ticker = Ticker::every(Duration::from_millis(5));
  loop {
    let mut raw = ButtonSet::empty();
    for (bit, input) in buttons.iter().enumerate() {
      if input.is_high() {
        raw |= ButtonSet::from_bits_truncate(1 << bit);
      }
    }
    let now = now_ms();
    let stable = debouncer.sample(raw, now, bench.debounce_ms);
    let chord = chords.update(stable, now, bench.chord_ms);
    let held = stable | ButtonSet::from_bits_truncate(CONSOLE_HELD.load(Ordering::Acquire));
    with_node(|node, io| {
      node.set_held(held);
      if let Some(chord) = chord {
        info!("buttons: {chord:?}");
        node.on_chord(now, chord, io);
      }
    });
    ticker.next().await;
  }
}

#[embassy_executor::task]
async fn clock() {
  let mut ticker = Ticker::every(Duration::from_millis(10));
  loop {
    let now = now_ms();
    with_node(|node, io| node.tick(now, io));
    ticker.next().await;
  }
}

#[embassy_executor::task]
async fn can(mut twai: Twai<'static, Async>) {
  loop {
    if twai.is_bus_off() {
      // Reference 5.3: wait 100 ms, then recover.
      warn!("CAN bus-off: recovering in 100 ms");
      Timer::after(Duration::from_millis(100)).await;
      twai.initiate_recovery();
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
            Err(error) => warn!("CAN frame rejected: {error}")
          }
        }
      }
      Either::First(Err(error)) => warn!("CAN receive error: {error:?}"),
      Either::Second(envelope) => {
        if let Message::Sensor(reading) = envelope.message {
          info!("SENSOR {reading:?}");
        }
        match envelope.encode::<EspTwaiFrame>() {
          Ok(frame) => match with_timeout(Duration::from_millis(20), twai.transmit_async(&frame)).await {
            Ok(Ok(())) => {}
            Ok(Err(error)) => warn!("CAN transmit failed: {error:?}"),
            Err(_) => warn!("CAN transmit timed out")
          },
          Err(error) => error!("encoding {:?} failed: {error}", envelope.message)
        }
      }
    }
  }
}

#[embassy_executor::task]
async fn console_task(mut rx: UsbSerialJtagRx<'static, Async>) {
  static LINE: StaticCell<[u8; 80]> = StaticCell::new();
  let line = LINE.init([0; 80]);
  let mut len = 0_usize;
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
        if !text.trim().is_empty() {
          info!("> {text}");
          execute(text);
        }
        len = 0;
      }
      other => {
        if let Some(slot) = line.get_mut(len) {
          *slot = other;
          len += 1;
        }
      }
    }
  }
}

fn execute(text: &str) {
  let command = match console::parse(text) {
    Ok(command) => command,
    Err(error) => {
      warn!("{error}");
      return;
    }
  };
  let now = now_ms();
  match command {
    Command::Help => {
      for line in console::HELP.lines() {
        info!("{line}");
      }
    }
    Command::Status => {
      with_node(|node, _| {
        info!(
          "state {:?}, held {:?}, reply delay {} ms",
          node.state(),
          node.held(),
          node.reply_delay_ms()
        );
        info!("injection {:?}", node.injection_mut());
        for seq in 0..=u8::MAX {
          if let Some(reading) = node.result(seq) {
            info!("kept: {reading:?}");
          }
        }
      });
    }
    Command::Bench(on) => {
      with_node(|node, _| node.set_bench(on));
      info!("bench mode {}", if on { "on" } else { "off" });
    }
    Command::Press(set) => {
      with_node(|node, io| node.on_chord(now, set, io));
      info!("press {set:?}");
    }
    Command::Hold(set) => {
      CONSOLE_HELD.store(set.bits(), Ordering::Release);
      info!("holding {set:?}");
    }
    Command::Release => {
      CONSOLE_HELD.store(0, Ordering::Release);
      info!("released");
    }
    Command::Shot { point, edge_age_10us } => {
      let seq = CONSOLE_SEQ.fetch_add(1, Ordering::AcqRel);
      let shot = Envelope {
        node: NodeId::COORDINATOR,
        message: Message::Shot(Shot {
          seq,
          point,
          edge_age_10us
        })
      };
      with_node(|node, io| node.on_frame(now, shot, io));
      info!("injected SHOT seq {seq} at {point:?}");
    }
    Command::Delay(ms) => {
      with_node(|node, _| node.set_reply_delay_ms(ms));
      info!("reply delay {ms} ms");
    }
    Command::Next(result) => {
      with_node(|node, _| node.injection_mut().next_result = Some(result));
      info!("next shot reports {result:?}");
    }
    Command::Sum(percent) => {
      with_node(|node, _| node.injection_mut().sum_q = percent.map(sum_q_from_percent));
      info!("sum {percent:?} percent");
    }
  }
}
