//! Every pin of the sensor node, in one place. PROVISIONAL: chosen for a Seeed XIAO ESP32C6 (owner, 2026-11-17), the
//! sensor's own board, with five buttons standing in for the detector (`docs/SENSOR_PLAN.md`, revision 4, sections
//! 7.3 and 7.5). Every pin used is on the XIAO's edge (D0 to D10); the XIAO name is in the table.
//!
//! Rules applied:
//! - No strapping pin (GPIO4, 5, 8, 9, 15): a button held at reset could change the boot. On the XIAO, GPIO 9 is its
//!   boot button and GPIO 15 its LED.
//! - GPIO12 and 13 stay with USB-Serial-JTAG (the console and the log), GPIO16 and 17 (D6, D7) with UART0.
//! - GPIO 3 and 14 run the XIAO's antenna switch, and stay in their reset state: the sensor uses no radio.
//! - GPIO 22 and 23 are TWAI transmit and receive, as on the motor node, ready for a transceiver.
//! - Buttons connect to 3V3 and the inputs are pulled down (active high): a broken wire reads as not pressed, and a
//!   ULN2003 input on the same pin, should this image ever run on a motor board, is held off rather than pulled on.
//!
//! | Signal | GPIO | XIAO | Direction | At boot |
//! |---|---|---|---|---|
//! | Button A, upper right | 18 | D10 | in, pull-down | not pressed |
//! | Button B, upper left | 19 | D8 | in, pull-down | not pressed |
//! | Button C, lower left | 20 | D9 | in, pull-down | not pressed |
//! | Button D, lower right | 21 | D3 | in, pull-down | not pressed |
//! | Button center | 2 | D2 | in, pull-down | not pressed |
//! | TWAI, one-wire loopback on the bench | 22 | D4 | open drain, pull-up | recessive |
//! | (TWAI receive with a transceiver) | 23 | D5 | unused on the bench | reset state |
//!
//! The center button was on GPIO 10 for a DevKitC-1; the XIAO does not bring GPIO 10 out.

use esp_hal::Blocking;
use esp_hal::gpio::{AnyPin, Flex};
use esp_hal::peripherals::TWAI0;
use esp_hal::twai::{BaudRate, TwaiConfiguration, TwaiMode};

/// The TWAI controller as this board wires it.
///
/// PROVISIONAL: the development board has no CAN transceiver, so TX and RX share GPIO 22, open drain with a pull-up.
/// Self-test mode needs no acknowledgement, so frames go out without a second node. GPIO 23 is left in reset.
/// With a transceiver, this becomes `TwaiConfiguration::new` with GPIO 22 and 23 and [`TwaiMode::Normal`].
pub fn twai(
  peripheral: TWAI0<'static>,
  tx: AnyPin<'static>,
  baud_rate: BaudRate
) -> TwaiConfiguration<'static, Blocking> {
  let (input, output) = Flex::new(tx).split();
  TwaiConfiguration::new_no_transceiver(peripheral, input, output, baud_rate, TwaiMode::SelfTest)
}

/// Takes the pins out of the peripherals by name, so that the GPIO numbers above appear exactly once in code.
#[macro_export]
macro_rules! board_pins {
  ($p:ident) => {
    $crate::board::Pins {
      buttons: [
        $p.GPIO18.into(),
        $p.GPIO19.into(),
        $p.GPIO20.into(),
        $p.GPIO21.into(),
        $p.GPIO2.into()
      ],
      twai_tx: $p.GPIO22.into()
    }
  };
}

/// The sensor's pins, type-erased.
pub struct Pins {
  /// Buttons A, B, C, D, and the center, in the order of [`sensor_logic::ButtonSet`]'s bits.
  pub buttons: [AnyPin<'static>; 5],
  /// TWAI transmit; on the bench also receive (see [`twai`]).
  pub twai_tx: AnyPin<'static>
}
