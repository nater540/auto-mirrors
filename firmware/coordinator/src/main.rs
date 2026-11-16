//! Coordinator firmware for the ESP32-C6: scaffold only (milestone M0).
//!
//! What this image does to hardware: nothing. It configures no GPIO, so every pin stays in its reset state.
//! It starts no bus. It boots, starts the Embassy time driver, logs its flash notice over USB-Serial-JTAG, and idles.
//!
//! `FLASH_NOTICE.txt`, beside this crate's `Cargo.toml`, states what the image does to motors and I/O.
//! `just flash` prints it and asks before flashing, and the image logs it at boot.
//! Change it in the same commit as any change to what this firmware drives.
//!
//! The orchestrator, the Ruida I/O, the interlock relay, and the console arrive in milestone M5,
//! with every pin in `board.rs`.
#![no_std]
#![no_main]
#![deny(
  clippy::mem_forget,
  reason = "mem::forget is not safe with esp-hal types that hold a buffer for the length of a transfer"
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use log::info;

/// What this image does to motors and I/O: the text `just flash` shows before flashing.
const FLASH_NOTICE: &str = include_str!("../FLASH_NOTICE.txt");

// The app descriptor the ESP-IDF second-stage bootloader requires in every image.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
  // No tasks yet.
  let _ = spawner;

  esp_println::logger::init_logger_from_env();

  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

  for line in FLASH_NOTICE.lines() {
    info!("{line}");
  }

  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}
