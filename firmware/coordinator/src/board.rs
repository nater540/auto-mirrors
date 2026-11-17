//! Every pin of the coordinator, in one place (hand-off 3.1). PROVISIONAL: chosen for bring-up on a second Adafruit
//! Feather ESP32-C6 on the bench, because the coordinator board does not exist yet (hand-off open item 7).
//! Replace this file when it does.
//!
//! Rules applied:
//! - The rules of the node's `board.rs`: no strapping pin (GPIO4, 5, 8, 9, 15) for a safety signal, GPIO12 and 13
//!   left to USB-Serial-JTAG, GPIO16 and 17 to UART0, and 18 to 20 to the Feather's I2C port.
//! - The bus pins are the node development board's, so either Feather takes either image without rewiring.
//! - **The two outputs that act on the machine are on pins seen to drive nothing on through reset and boot.** On the
//!   node's Feather, GPIO 22 and 3 drive ULN2003 inputs, and no coil LED lit while the reset button was held or the
//!   board booted (the node's `board.rs`, 2026-11-17). That is the load that matters here too: a relay driver and
//!   FootSW's optocoupler need milliamperes, which a weak internal pull-up would not supply either. The relay drive
//!   is GPIO 22; FootSW, which starts a laser job when held for 100 ms, is GPIO 3.
//! - The relay's feedback is on GPIO 21, beside its drive, so that one jumper closes the bench loop.
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
//! | FootSW, to the optocoupler that pulls the Ruida's FootSW low | 3 | A4 | out, high = active | low: inactive |
//! | Ruida Status: job running | 2 | A5 | in, pull-up, low = running | reads no job |
//! | Fire sense: laser firing | 7 | 7 | in, pull-up, low = firing | reads no fire |
//!
//! On the bench there is no relay and no Ruida: MO is jumpered to SCK, so the feedback follows the drive, and MO
//! also drives the node's permit input in place of its 3V jumper. A4 drives nothing; A5 and 7 stay open (no job,
//! no fire). Touching one of them to GND stands in for the Ruida.

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
      footsw: $p.GPIO3.into(),
      status: $p.GPIO2.into(),
      fire: $p.GPIO7.into()
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
  /// FootSW's optocoupler, high to start the job.
  pub footsw: AnyPin<'static>,
  /// The Ruida's Status output, low while a job runs.
  pub status: AnyPin<'static>,
  /// The fire sense, low while the laser fires.
  pub fire: AnyPin<'static>
}
