//! The pin table of the board the image is built for (hand-off 3.1): every pin of the node, and the notice that says
//! what the image drives, in one file per board.
//!
//! - [`feather`]: an Adafruit Feather ESP32-C6, one motor channel. The default.
//! - [`devkitc`]: an Espressif ESP32-C6-DevKitC-1, two motor channels, built with `--features devkitc`
//!   (`just flash node-devkitc`).
//!
//! Each file gives the same items: `FLASH_NOTICE`, `NODE_ID`, `CHANNELS`, `POPULATED_CHANNELS`, `twai`, `Pins`, and
//! the `board_pins!` macro. Both are PROVISIONAL until the node board exists (hand-off open item 7).

#[cfg(feature = "devkitc")]
mod devkitc;
#[cfg(not(feature = "devkitc"))]
mod feather;

#[cfg(feature = "devkitc")]
pub use devkitc::*;
#[cfg(not(feature = "devkitc"))]
pub use feather::*;
