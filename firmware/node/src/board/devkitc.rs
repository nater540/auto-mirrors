//! Every pin of the node on an Espressif ESP32-C6-DevKitC-1, in one place (hand-off 3.1). PROVISIONAL: chosen for
//! a two-channel node on the bench, because the node board does not exist yet (hand-off open item 7). The image built
//! with the `devkitc` feature uses it (`just flash node-devkitc`).
//!
//! The header positions are from Espressif's user guide of the ESP32-C6-DevKitC-1 (esp-dev-kits, read 2026-11-19),
//! not yet checked against the owner's board.
//!
//! Rules applied:
//! - No strapping pin (GPIO4, 5, 8, 9, 15) carries a safety signal or a coil (reference 7.4). On this board GPIO8
//!   also drives the addressable RGB LED, and 9 is the Boot button.
//! - GPIO12 and 13 stay with USB-Serial-JTAG (the console and the log; the USB port, not the UART port), GPIO16
//!   and 17 with UART0: the ROM writes its boot log on GPIO16, which would twitch a coil.
//! - That leaves fourteen header pins: GPIO 6, 7, 0, 1, 10, 11, 2, 3 on J1 and 23 to 18 on J3. A two-channel node
//!   uses thirteen; GPIO1 is spare.
//! - Each channel's four coils are adjacent on one header, so one ribbon of four wires reaches its IN1 to IN4.
//! - The bus, the two inputs, and the arm pulse keep the Feather's GPIOs where this board has them (23, 6, 7, 0);
//!   the transceiver's RX moves to 22, beside TX, so that its two signal wires are adjacent.
//! - The two inputs are pulled down, so a broken wire reads as no permit and no rail.
//!
//! | Signal | GPIO | Header pin | Direction | At boot |
//! |---|---|---|---|---|
//! | Channel 0, IN1 to IN4 (blue, pink, yellow, orange) | 21, 20, 19, 18 | J3 7, 8, 9, 10 | out | low: coils off |
//! | Channel 1, IN1 to IN4 (blue, pink, yellow, orange) | 10, 11, 2, 3 | J1 10, 11, 12, 13 | out | low: coils off |
//! | Channel 2 | none | | | unpopulated (addendum A.7) |
//! | TWAI TX, to the transceiver's CTX | 23 | J3 5 | out | reset state until TWAI starts |
//! | TWAI RX, from the transceiver's CRX | 22 | J3 6 | in, no pull | driven by the transceiver |
//! | Motor permit | 6 | J1 5 | in, pull-down | absent |
//! | Rail sense (`rail_ok`) | 7 | J1 6 | in, pull-down | down |
//! | Arm pulse to the charge pump | 0 | J1 7 | out | low: not armed |
//!
//! UNVERIFIED on this board, each to check before a motor is connected:
//! - That no coil is driven on between reset and the firmware's first instruction. On the Feather (2026-11-17) none
//!   of the ULN2003's LEDs lit through reset and boot; GPIO 18 to 21 and 10 and 11 were not among the Feather's coil
//!   pins, so this board must be watched the same way, on both channels.
//! - That no 32 kHz crystal is fitted on GPIO0 and 1 (`XTAL_32K_P` and `_N`). The user guide names the function but
//!   not a crystal; one fitted would hold the arm pulse's pin.
//! - Whether the transceiver's CTX input floats dominant while GPIO 23 is in its reset state, as on the Feather.

use align_proto::NodeId;
use esp_hal::Blocking;
use esp_hal::gpio::AnyPin;
use esp_hal::peripherals::TWAI0;
use esp_hal::twai::{BaudRate, TwaiConfiguration, TwaiMode};

/// What this image does to motors and I/O, logged at boot and acknowledged before flashing.
pub const FLASH_NOTICE: &str = include_str!("../../FLASH_NOTICE-devkitc.txt");

/// PROVISIONAL: this board answers as the mirror 2 node, the mirror whose horizontal screw has the bench motor
/// (owner, 2026-11-18), so that the coordinator's `m2h` and `m2v` reach it. The Feather answers as mirror 1, so both
/// can share a bus. A second DevKitC-1 needs its own address.
pub const NODE_ID: NodeId = NodeId::MIRROR_2;

/// Channels with a motor connector wired: 0 and 1 (see the rules above).
pub const CHANNELS: usize = 2;

/// [`CHANNELS`], as the node counts them.
pub const POPULATED_CHANNELS: u8 = 2;

const _: () = assert!(
  CHANNELS == POPULATED_CHANNELS as usize,
  "the pin table and the node agree on the channels"
);

/// The TWAI controller as this board wires it: through a transceiver, in [`TwaiMode::Normal`].
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
      channels: [
        [$p.GPIO21.into(), $p.GPIO20.into(), $p.GPIO19.into(), $p.GPIO18.into()],
        [$p.GPIO10.into(), $p.GPIO11.into(), $p.GPIO2.into(), $p.GPIO3.into()]
      ],
      twai_tx: $p.GPIO23.into(),
      twai_rx: $p.GPIO22.into(),
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
  /// TWAI transmit, to the transceiver.
  pub twai_tx: AnyPin<'static>,
  /// TWAI receive, from the transceiver.
  pub twai_rx: AnyPin<'static>,
  /// The hardware motor permit from the drag chain, high when present.
  pub permit: AnyPin<'static>,
  /// The switched motor rail through a divider, high when up.
  pub rail_sense: AnyPin<'static>,
  /// The 1 kHz arm pulse to the charge pump.
  pub arm: AnyPin<'static>
}
