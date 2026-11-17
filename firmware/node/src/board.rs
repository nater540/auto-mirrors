//! Every pin of the node, in one place (hand-off 3.1). PROVISIONAL: chosen for bring-up on the development board on
//! the bench, an Adafruit Feather ESP32-C6, because the node board does not exist yet (hand-off open item 7).
//! Replace this file when it does.
//!
//! Rules applied:
//! - No strapping pin (GPIO4, 5, 8, 9, 15) carries a safety signal or a coil (reference 7.4). A ULN2003 input on a
//!   strapping pin could also pull it at reset and change the boot mode. On the Feather, 9 is also the boot button
//!   and the RGB LED's data line, and 15 the red LED.
//! - GPIO12 and 13 stay with USB-Serial-JTAG (the console and the log), GPIO16 and 17 with UART0: the ROM writes its
//!   boot log on GPIO16, which would twitch a coil.
//! - The Feather's own circuits keep their pins: GPIO20 switches the power of the RGB LED and the STEMMA QT port,
//!   and GPIO18 and 19 are its I2C pins (SCL, SDA), shared with that port. GPIO10 and 11 are not on its headers.
//! - That leaves ten header pins (0, 1, 2, 3, 6, 7, 14, 21, 22, 23): enough for one channel, the bus, the two inputs,
//!   and the arm pulse, not for a second channel. Channel 1 is unpopulated on this board.
//! - The four coils are adjacent on the left header (A4, A5, SCK, MO), so one ribbon of four wires reaches IN1 to IN4.
//! - The two inputs are pulled down, so a broken wire reads as no permit and no rail.
//!
//! | Signal | GPIO | Feather label | Direction | At boot |
//! |---|---|---|---|---|
//! | Channel 0, IN1 to IN4 (blue, pink, yellow, orange) | 3, 2, 21, 22 | A4, A5, SCK, MO | out | low: coils off |
//! | Channel 1 | none | | | unpopulated on this board |
//! | Channel 2 | none | | | unpopulated (addendum A.7) |
//! | TWAI, one-wire loopback on the bench | 23 | MI | open drain, pull-up | recessive |
//! | Motor permit | 6 | A2 (left), 6 (right) | in, pull-down | absent |
//! | Rail sense (`rail_ok`) | 7 | 7 | in, pull-down | down |
//! | Arm pulse to the charge pump | 0 | 0 | out | low: not armed |
//!
//! VERIFIED on the bench (2026-11-17): no coil pin is pulled up between reset and the firmware's first instruction.
//! None of the ULN2003's four LEDs lit while the reset button was held or while the board booted.
//! Repeat the check on every new board.

use align_proto::NodeId;
use esp_hal::Blocking;
use esp_hal::gpio::{AnyPin, Flex};
use esp_hal::peripherals::TWAI0;
use esp_hal::twai::{BaudRate, TwaiConfiguration, TwaiMode};

/// PROVISIONAL: this board answers as the mirror 1 node. A second node board needs its own address.
pub const NODE_ID: NodeId = NodeId::MIRROR_1;

/// Channels with a motor connector wired: channel 0 only (see the rules above).
pub const CHANNELS: usize = 1;

/// [`CHANNELS`], as the node counts them.
pub const POPULATED_CHANNELS: u8 = 1;

const _: () = assert!(
  CHANNELS == POPULATED_CHANNELS as usize,
  "the pin table and the node agree on the channels"
);

/// The TWAI controller as this board wires it.
///
/// PROVISIONAL: the development board has no CAN transceiver, so TX and RX share GPIO 23, open drain with a pull-up.
/// The controller reads back every bit it sends, as on a bus with one node. Self-test mode needs no acknowledgement,
/// so the node's frames go out and the controller never reaches bus-off.
/// A node board with a transceiver uses `TwaiConfiguration::new` with two pins and [`TwaiMode::Normal`].
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
      channels: [[$p.GPIO3.into(), $p.GPIO2.into(), $p.GPIO21.into(), $p.GPIO22.into()]],
      twai_tx: $p.GPIO23.into(),
      permit: $p.GPIO6.into(),
      rail_sense: $p.GPIO7.into(),
      arm: $p.GPIO0.into()
    }
  };
}

/// The node's pins, type-erased.
pub struct Pins {
  /// IN1 to IN4 of the ULN2003 of each populated channel, channel 0 first.
  pub channels: [[AnyPin<'static>; 4]; CHANNELS],
  /// TWAI transmit; on the bench also receive (see [`twai`]).
  pub twai_tx: AnyPin<'static>,
  /// The hardware motor permit from the drag chain, high when present.
  pub permit: AnyPin<'static>,
  /// The switched motor rail through a divider, high when up.
  pub rail_sense: AnyPin<'static>,
  /// The 1 kHz arm pulse to the charge pump.
  pub arm: AnyPin<'static>
}
