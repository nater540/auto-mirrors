//! A simulated Ruida controller on the loopback interface, so the probe runs without a laser.
//!
//! AS MODELED from reference 6.1 and 6.4 and MeerK40t, and verified on nothing: it checks a packet's checksum at its
//! own basis and answers a mismatch with NAK; it answers a read of one of the five addresses with an ACK and then
//! the answer, each in its own datagram, swizzled at the 644XG magic; and it answers anything else with ERR, where a
//! real controller would run it (reference 6.0). It keeps every command it was sent, unswizzled, so a test can show
//! the probe sent nothing but reads. It answers whoever sent the packet, not port 40200.

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use ruida_proto::{ChecksumBasis, MAGIC_644XG, Query, encode_i35, encode_u35, swizzle, unswizzle};

/// The unswizzled one-byte answers (reference 6.1).
const ACK: u8 = 0xCC;
const ERR: u8 = 0xCD;
const NAK: u8 = 0xCF;

/// How the simulated controller answers a packet whose checksum it accepts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Behavior {
  /// An ACK, then the answer.
  Answer,
  /// An ACK, then the answer at another address (a late answer to something else), then the answer.
  StrayFirst,
  /// An ACK and nothing more.
  AckOnly,
  /// ERR.
  Refuse,
  /// Nothing, to any packet.
  Silent
}

/// The simulated controller: what it reads back and how it answers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SimController {
  /// The checksum basis it checks packets at.
  pub basis: ChecksumBasis,
  /// How it answers an accepted packet.
  pub behavior: Behavior,
  /// The card ID it reports: the EduTech Wiki's 0x65006500 by default.
  pub card_id: u64,
  /// The mainboard version it reports.
  pub mainboard_version: u64,
  /// The status word it reports.
  pub status: u64,
  /// The X position it reports, in micrometers.
  pub x_um: i64,
  /// The Y position it reports, in micrometers.
  pub y_um: i64,
  /// Whether an answer carries a second value, as reference 6.4 shows (MeerK40t reads one).
  pub second_value: bool,
  /// Every command it was sent, unswizzled, after its two checksum bytes.
  received: Vec<Vec<u8>>,
  /// Every datagram it was sent, as sent.
  received_wire: Vec<Vec<u8>>
}

impl SimController {
  /// A controller that checks the checksum at `basis` and answers every read, idle at a position off zero.
  pub fn new(basis: ChecksumBasis) -> Self {
    Self {
      basis,
      behavior: Behavior::Answer,
      card_id: 0x6500_6500,
      mainboard_version: 0x0003_0201,
      status: 0,
      x_um: 125_000,
      y_um: -2_500,
      second_value: false,
      received: Vec::new(),
      received_wire: Vec::new()
    }
  }

  /// Every command it was sent, unswizzled at the 644XG magic, without the checksum.
  pub fn received(&self) -> &[Vec<u8>] {
    &self.received
  }

  /// Every datagram it was sent, as it arrived.
  pub fn received_wire(&self) -> &[Vec<u8>] {
    &self.received_wire
  }

  /// The datagrams it sends back for one it receives, in order.
  pub fn answer(&mut self, wire: &[u8]) -> Vec<Vec<u8>> {
    self.received_wire.push(wire.to_vec());
    let (sum, command_wire) = match wire {
      [high, low, rest @ ..] => (u16::from_be_bytes([*high, *low]), rest),
      _ => return Vec::new()
    };
    let command: Vec<u8> = command_wire.iter().map(|&byte| unswizzle(byte, MAGIC_644XG)).collect();
    self.received.push(command.clone());
    if self.behavior == Behavior::Silent {
      return Vec::new();
    }
    if self.basis.checksum(&command, command_wire) != sum {
      return vec![one_byte(NAK)];
    }
    let query = match command.as_slice() {
      [0xDA, 0x00, high, low] => Query::at([*high, *low]),
      _ => None
    };
    let Some(query) = query else {
      return vec![one_byte(ERR)];
    };
    match self.behavior {
      Behavior::Answer => vec![one_byte(ACK), self.answer_to(query)],
      Behavior::StrayFirst => {
        let other = if query == Query::Status {
          Query::PositionX
        } else {
          Query::Status
        };
        vec![one_byte(ACK), self.answer_to(other), self.answer_to(query)]
      }
      Behavior::AckOnly => vec![one_byte(ACK)],
      Behavior::Refuse | Behavior::Silent => vec![one_byte(ERR)]
    }
  }

  /// `DA 01`, the query's address, and its value, swizzled.
  fn answer_to(&self, query: Query) -> Vec<u8> {
    let value = match query {
      Query::CardId => encode_u35(self.card_id),
      Query::MainboardVersion => encode_u35(self.mainboard_version),
      Query::Status => encode_u35(self.status),
      Query::PositionX => encode_i35(self.x_um),
      Query::PositionY => encode_i35(self.y_um)
    }
    .unwrap_or([0x7F; 5]);
    let [high, low] = query.address();
    let mut plain = vec![0xDA, 0x01, high, low];
    plain.extend(value);
    if self.second_value {
      plain.extend(value);
    }
    plain.into_iter().map(|byte| swizzle(byte, MAGIC_644XG)).collect()
  }
}

/// A one-byte answer, swizzled.
fn one_byte(plain: u8) -> Vec<u8> {
  vec![swizzle(plain, MAGIC_644XG)]
}

/// A simulated controller answering on a loopback socket in its own thread.
pub struct Running {
  address: SocketAddr,
  stop: Arc<AtomicBool>,
  thread: JoinHandle<io::Result<SimController>>
}

impl Running {
  /// The address to send its packets to.
  pub fn address(&self) -> SocketAddr {
    self.address
  }

  /// Stop it, and get the controller back with what it received.
  ///
  /// # Errors
  ///
  /// The socket error that ended it early, or [`io::ErrorKind::Other`] if its thread panicked.
  pub fn stop(self) -> io::Result<SimController> {
    self.stop.store(true, Ordering::Relaxed);
    self
      .thread
      .join()
      .map_err(|_| io::Error::other("the simulated controller's thread panicked"))?
  }
}

/// Start `controller` answering on a free loopback port.
///
/// # Errors
///
/// The socket could not be bound or set up.
pub fn serve(mut controller: SimController) -> io::Result<Running> {
  let socket = UdpSocket::bind("127.0.0.1:0")?;
  socket.set_read_timeout(Some(Duration::from_millis(10)))?;
  let address = socket.local_addr()?;
  let stop = Arc::new(AtomicBool::new(false));
  let stopped = Arc::clone(&stop);
  let thread = thread::spawn(move || {
    let mut buffer = [0_u8; 2048];
    while !stopped.load(Ordering::Relaxed) {
      match socket.recv_from(&mut buffer) {
        Ok((len, from)) => {
          for datagram in controller.answer(buffer.get(..len).unwrap_or_default()) {
            socket.send_to(&datagram, from)?;
          }
        }
        Err(error) if matches!(error.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
        Err(error) => return Err(error)
      }
    }
    Ok(controller)
  });
  Ok(Running { address, stop, thread })
}

#[cfg(test)]
mod tests {
  use ruida_proto::{Datagram, Reading, decode_datagram};

  use super::*;

  fn decoded(datagrams: &[Vec<u8>]) -> Vec<Datagram> {
    datagrams
      .iter()
      .map(|datagram| decode_datagram(datagram).unwrap())
      .collect()
  }

  fn read(datagram: &Datagram, query: Query) -> Reading {
    match *datagram {
      Datagram::Answer { address, value, .. } if address == query.address() => Reading::of(query, value),
      ref other => panic!("expected the answer to {query:?}, got {other:?}")
    }
  }

  #[test]
  fn a_read_at_the_right_basis_is_acked_then_answered() {
    let mut controller = SimController::new(ChecksumBasis::Swizzled);
    for (query, expected) in [
      (Query::CardId, Reading::CardId { id: 0x6500_6500 }),
      (Query::PositionX, Reading::PositionX { x_um: 125_000 }),
      (Query::PositionY, Reading::PositionY { y_um: -2_500 })
    ] {
      let answers = decoded(&controller.answer(query.packet(ChecksumBasis::Swizzled).bytes()));
      assert_eq!(answers.len(), 2);
      assert_eq!(answers[0], Datagram::Ack);
      assert_eq!(read(&answers[1], query), expected);
    }
    assert_eq!(controller.received().len(), 3);
    assert_eq!(controller.received()[0], Query::CardId.plain());
    assert_eq!(
      controller.received_wire()[0],
      Query::CardId.packet(ChecksumBasis::Swizzled).bytes()
    );
  }

  #[test]
  fn a_read_at_the_other_basis_is_refused_with_a_nak() {
    let mut controller = SimController::new(ChecksumBasis::Plain);
    let answers = decoded(&controller.answer(Query::Status.packet(ChecksumBasis::Swizzled).bytes()));
    assert_eq!(answers, [Datagram::Nak]);
    let answers = decoded(&controller.answer(Query::Status.packet(ChecksumBasis::Plain).bytes()));
    assert_eq!(answers[0], Datagram::Ack);
  }

  #[test]
  fn anything_but_a_read_of_the_five_addresses_gets_an_err() {
    let mut controller = SimController::new(ChecksumBasis::Plain);
    // D8 00 is Start (reference 6.4), with a correct plain checksum: the controller would run it.
    let start = [0x00, 0xD8, swizzle(0xD8, MAGIC_644XG), swizzle(0x00, MAGIC_644XG)];
    assert_eq!(decoded(&controller.answer(&start)), [Datagram::Err]);
    assert_eq!(controller.received(), [vec![0xD8, 0x00]]);
  }

  #[test]
  fn a_datagram_too_short_for_a_checksum_gets_no_answer() {
    let mut controller = SimController::new(ChecksumBasis::Swizzled);
    assert!(controller.answer(&[0x01]).is_empty());
    assert!(controller.received().is_empty());
    assert_eq!(controller.received_wire().len(), 1);
  }

  #[test]
  fn each_behavior_answers_its_own_way() {
    let packet = Query::Status.packet(ChecksumBasis::Swizzled);
    let answers_of = |behavior: Behavior| {
      let mut controller = SimController::new(ChecksumBasis::Swizzled);
      controller.behavior = behavior;
      decoded(&controller.answer(packet.bytes()))
    };
    let stray = answers_of(Behavior::StrayFirst);
    assert_eq!(stray[0], Datagram::Ack);
    assert_eq!(read(&stray[1], Query::PositionX), Reading::PositionX { x_um: 125_000 });
    assert!(matches!(read(&stray[2], Query::Status), Reading::Status(_)));
    assert_eq!(answers_of(Behavior::AckOnly), [Datagram::Ack]);
    assert_eq!(answers_of(Behavior::Refuse), [Datagram::Err]);
    assert!(answers_of(Behavior::Silent).is_empty());
  }

  #[test]
  fn a_second_value_repeats_the_first() {
    let mut controller = SimController::new(ChecksumBasis::Swizzled);
    controller.second_value = true;
    let answers = decoded(&controller.answer(Query::CardId.packet(ChecksumBasis::Swizzled).bytes()));
    let Datagram::Answer { value, second, .. } = answers[1] else {
      panic!("expected an answer, got {:?}", answers[1]);
    };
    assert_eq!(second, Some(value));
  }
}
