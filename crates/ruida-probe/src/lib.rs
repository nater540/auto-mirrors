//! The read-only first test of the Ruida link (`docs/REFERENCE.md` section 6.5, hand-off M7), run from a computer
//! on the laser's network rather than the coordinator, which has no network yet.
//!
//! - [`probe`]: the five memory reads once, then the status and position polled if asked, every datagram either way
//!   logged as an `RU1` line with its bytes as sent and as meant.
//! - [`sim`]: a simulated controller on the loopback interface, AS MODELED from reference 6.1 and 6.4, which the
//!   tests and `ruida-probe --sim` run the probe against.
//!
//! The probe sends nothing but [`ruida_proto::Query::packet`]s: the protocol crate can build no other packet.

pub mod probe;
pub mod sim;
