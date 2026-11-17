//! Every pin of the coordinator, in one place (hand-off 3.1). PROVISIONAL: chosen for bring-up on a second Adafruit
//! Feather ESP32-C6 on the bench, because the coordinator board does not exist yet (hand-off open item 7).
//! Replace this file when it does.
//!
//! So far only the bus. The relay drive and its feedback, the FootSW output, and the Status and fire sense inputs
//! (reference 6.6 and 7.1) get pins with the state machine that uses them. Until then every other GPIO stays in its
//! reset state.
//!
//! The bus pins are the node development board's, so that either Feather takes either image without rewiring its
//! transceiver. The rules of the node's `board.rs` apply here too: no strapping pin (GPIO4, 5, 8, 9, 15) for a safety
//! signal, GPIO12 and 13 left to USB-Serial-JTAG, GPIO16 and 17 to UART0, and 18 to 20 to the Feather's I2C port.
//!
//! | Signal | GPIO | Feather label | Direction | At boot |
//! |---|---|---|---|---|
//! | TWAI TX, to the transceiver's CTX | 23 | MI | out | reset state until TWAI starts |
//! | TWAI RX, from the transceiver's CRX | 1 | A0 | in, no pull | driven by the transceiver |

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
      twai_rx: $p.GPIO1.into()
    }
  };
}

/// The coordinator's pins, type-erased.
pub struct Pins {
  /// TWAI transmit, to the transceiver.
  pub twai_tx: AnyPin<'static>,
  /// TWAI receive, from the transceiver.
  pub twai_rx: AnyPin<'static>
}
