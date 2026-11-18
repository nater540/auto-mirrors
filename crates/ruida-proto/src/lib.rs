//! The Ruida controller's UDP protocol, read-only (`docs/REFERENCE.md` section 6, hand-off milestone M7): the
//! swizzle, the checksum, the queries this system may send, and the reading of the controller's replies.
//!
//! **Only reads can be built.** The one way to make a packet is [`Query::packet`], and a [`Query`] is one of five
//! memory reads (`DA 00` and an address): the card ID, the mainboard version, the machine status, and the X and Y
//! positions. Nothing here encodes a move, a job, or a key press, and nothing may until the owner confirms that the
//! read-only test of reference 6.5 passed on the Mod5 (hand-off M7).
//!
//! **The magic is fixed at 0x88**, the RDC644XG family's (reference 6.2). The controller acts on whatever it
//! unswizzles (reference 6.0: no arbitration), and a checksum summed over the wire bytes is right whatever the magic,
//! so a query swizzled with another magic reaches the controller as other bytes, command bytes among them. The tests
//! of [`packet`] show what each query becomes.
//!
//! - [`mod@swizzle`]: the byte scrambling.
//! - [`packet`]: the queries, the checksum and its two candidate bases, and what a datagram from the controller says.
//! - [`value`]: the 35-bit values, the machine status bits, and the reading of each query.
//! - [`exchange`]: one query from its packet to its reading, without I/O: the NAK retry, the timeout, strays.
//!
//! The crate is `no_std` without `alloc`, so the coordinator can run it once it has a network (reference 5.6).
#![no_std]

pub mod exchange;
pub mod packet;
pub mod swizzle;
pub mod value;

pub use exchange::{Exchange, Failure, Limits, Outcome, Step};
pub use packet::{
  CONTROLLER_PORT, ChecksumBasis, Datagram, DatagramError, PACKET_LEN, Packet, Query, REPLY_PORT, decode_datagram
};
pub use swizzle::{MAGIC_644XG, swizzle, unswizzle};
pub use value::{MachineStatus, Reading, decode_i35, decode_u35, encode_i35, encode_u35};
