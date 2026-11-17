//! Every pin of the coordinator, in one place (hand-off 3.1). PROVISIONAL: chosen for bring-up on a second Adafruit
//! Feather ESP32-C6 on the bench, because the coordinator board does not exist yet (hand-off open item 7).
//! Replace this file when it does.
//!
//! Rules applied:
//! - The rules of the node's `board.rs`: no strapping pin (GPIO4, 5, 8, 9, 15) for a safety signal, GPIO12 and 13
//!   left to USB-Serial-JTAG, GPIO16 and 17 to UART0, and 18 to 20 to the Feather's I2C port.
//! - The bus pins are the node development board's, so either Feather takes either image without rewiring.
//! - The relay drive is on GPIO 22, which on the node's Feather drives a coil and was seen to stay low through reset
//!   and boot (the node's `board.rs`, 2026-11-17). Its feedback is on the neighboring GPIO 21, so that one jumper
//!   closes the bench loop.
//! - **The feedback reads high when the laser loop is open**, through the relay's normally open auxiliary contact to
//!   3V, with a pull-down. A broken wire then reads "closed": a move is never armed on it, and a loop that drops
//!   while moving faults. Sensing the normally closed contact instead would let a broken wire read "open" and arm.
//! - Status and fire sense are active low with pull-ups, as an optocoupler's transistor pulls them (reference 6.6).
//!   A broken wire reads as no job and no fire. Their polarity on the Mod5 is UNVERIFIED (hand-off open item 9).
//!
//! | Signal | GPIO | Feather label | Direction | At boot |
//! |---|---|---|---|---|
//! | TWAI TX, to the transceiver's CTX | 23 | MI | out | reset state until TWAI starts |
//! | TWAI RX, from the transceiver's CRX | 1 | A0 | in, no pull | driven by the transceiver |
//! | Relay drive (`ALIGN_MOVE`, reference 7.1) | 22 | MO | out, high energizes | low: released |
//! | Relay feedback: laser loop open | 21 | SCK | in, pull-down, high = open | reads closed |
//! | Ruida Status: job running | 3 | A4 | in, pull-up, low = running | reads no job |
//! | Fire sense: laser firing | 2 | A5 | in, pull-up, low = firing | reads no fire |
//!
//! On the bench there is no relay and no Ruida: MO is jumpered to SCK, so the feedback follows the drive, and MO
//! also drives the node's permit input in place of its 3V jumper. A4 and A5 stay open (no job, no fire); touching
//! one to GND stands in for the Ruida.

use esp_hal::Blocking;
use esp_hal::gpio::AnyPin;
use esp_hal::peripherals::TWAI0;
use esp_hal::twai::{BaudRate, TwaiConfiguration, TwaiMode};

/// The TWAI controller as this board wires it: through an SN65HVD230 transceiver, in [`TwaiMode::Normal`].
///
/// Every frame then needs another node on the bus to acknowledge it. Alone on the bus, the controller turns error
/// passive and esp-hal aborts each frame (`EspTwaiError::TransmissionAborted`); it never reaches bus-off that way.
pub fn twai(
  peripheral: TWAI0<'static>,
  tx: AnyPin<'static>,
  rx: AnyPin<'static>,
  baud_rate: BaudRate
) -> TwaiConfiguration<'static, Blocking> {
  TwaiConfiguration::new(peripheral, rx, tx, baud_rate, TwaiMode::Normal)
}

/// Takes the pins out of the peripherals by name, so that the GPIO numbers above appear exactly once in code.
#[macro_export]
macro_rules! board_pins {
  ($p:ident) => {
    $crate::board::Pins {
      twai_tx: $p.GPIO23.into(),
      twai_rx: $p.GPIO1.into(),
      relay: $p.GPIO22.into(),
      relay_feedback: $p.GPIO21.into(),
      status: $p.GPIO3.into(),
      fire: $p.GPIO2.into()
    }
  };
}

/// The coordinator's pins, type-erased.
pub struct Pins {
  /// TWAI transmit, to the transceiver.
  pub twai_tx: AnyPin<'static>,
  /// TWAI receive, from the transceiver.
  pub twai_rx: AnyPin<'static>,
  /// The relay drive, high to energize.
  pub relay: AnyPin<'static>,
  /// The relay's feedback, high while the laser loop is open.
  pub relay_feedback: AnyPin<'static>,
  /// The Ruida's Status output, low while a job runs.
  pub status: AnyPin<'static>,
  /// The fire sense, low while the laser fires.
  pub fire: AnyPin<'static>
}
