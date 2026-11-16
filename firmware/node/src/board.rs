//! Every pin of the node, in one place (hand-off 3.1). PROVISIONAL: chosen for bring-up on a bare ESP32-C6
//! development board, because the node board does not exist yet (hand-off open item 7). Replace this file when it does.
//!
//! Rules applied:
//! - No strapping pin (GPIO4, 5, 8, 9, 15) carries a safety signal or a coil (reference 7.4). A ULN2003 input on a
//!   strapping pin could also pull it at reset and change the boot mode.
//! - GPIO12 and 13 stay with USB-Serial-JTAG (the console and the log), GPIO16 and 17 with UART0.
//! - Only pins on both headers of the ESP32-C6-DevKitC-1 are used (GPIO14 is not on its headers).
//! - The two inputs are pulled down, so a broken wire reads as no permit and no rail.
//!
//! | Signal | GPIO | Direction | At boot |
//! |---|---|---|---|
//! | Channel 0, IN1 to IN4 (blue, pink, yellow, orange) | 18, 19, 20, 21 | out | low: coils off |
//! | Channel 1, IN1 to IN4 | 10, 11, 2, 3 | out | low: coils off |
//! | Channel 2 | none | | unpopulated (addendum A.7) |
//! | TWAI, one-wire loopback on the bench | 22 | open drain, pull-up | recessive |
//! | (TWAI RX on a node board) | 23 | unused on the bench | reset state |
//! | Motor permit | 6 | in, pull-down | absent |
//! | Rail sense (`rail_ok`) | 7 | in, pull-down | down |
//! | Arm pulse to the charge pump | 0 | out | low: not armed |

use align_proto::NodeId;
use esp_hal::Blocking;
use esp_hal::gpio::{AnyPin, Flex};
use esp_hal::peripherals::TWAI0;
use esp_hal::twai::{BaudRate, TwaiConfiguration, TwaiMode};

/// PROVISIONAL: this board answers as the mirror 1 node. A second node board needs its own address.
pub const NODE_ID: NodeId = NodeId::MIRROR_1;

/// Channels with a motor connector wired: 0 and 1.
pub const POPULATED_CHANNELS: u8 = 2;

/// The TWAI controller as this board wires it.
///
/// PROVISIONAL: the development board has no CAN transceiver, so TX and RX share GPIO 22, open drain with a pull-up.
/// The controller reads back every bit it sends, as on a bus with one node. Self-test mode needs no acknowledgement,
/// so the node's frames go out and the controller never reaches bus-off. GPIO 23 is left in its reset state.
/// A node board with a transceiver uses `TwaiConfiguration::new` with both pins and [`TwaiMode::Normal`].
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
      channel_0: [$p.GPIO18.into(), $p.GPIO19.into(), $p.GPIO20.into(), $p.GPIO21.into()],
      channel_1: [$p.GPIO10.into(), $p.GPIO11.into(), $p.GPIO2.into(), $p.GPIO3.into()],
      twai_tx: $p.GPIO22.into(),
      permit: $p.GPIO6.into(),
      rail_sense: $p.GPIO7.into(),
      arm: $p.GPIO0.into()
    }
  };
}

/// The node's pins, type-erased.
pub struct Pins {
  /// IN1 to IN4 of the ULN2003 on channel 0.
  pub channel_0: [esp_hal::gpio::AnyPin<'static>; 4],
  /// IN1 to IN4 of the ULN2003 on channel 1.
  pub channel_1: [esp_hal::gpio::AnyPin<'static>; 4],
  /// TWAI transmit; on the bench also receive (see [`twai`]). A node board adds GPIO 23 as receive.
  pub twai_tx: esp_hal::gpio::AnyPin<'static>,
  /// The hardware motor permit from the drag chain, high when present.
  pub permit: esp_hal::gpio::AnyPin<'static>,
  /// The switched motor rail through a divider, high when up.
  pub rail_sense: esp_hal::gpio::AnyPin<'static>,
  /// The 1 kHz arm pulse to the charge pump.
  pub arm: esp_hal::gpio::AnyPin<'static>
}
