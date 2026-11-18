//! One query, from its packet to its reading, without I/O. The caller sends each packet it is given, passes in every
//! datagram from the controller with the time, and drops the exchange once it returns [`Step::Done`].
//!
//! The rules (reference 6.1 and 6.5; MeerK40t `udp_connection.py`):
//!
//! - The controller accepts a packet with an ACK, then sends the answer in a datagram of its own. An answer at the
//!   query's address ends the exchange whether or not its ACK was seen, and whenever it arrives: an answer shows that
//!   the packet was accepted.
//! - A NAK asks for the packet again. While the controller's checksum basis is not known, the packet goes again
//!   summed the other way (reference 6.5 step 3); once it is known, the packet goes again unchanged, as MeerK40t
//!   resends. A NAK after [`Limits::tries`] packets ends the exchange.
//! - An ERR ends it.
//! - A datagram that is an answer at another address, an ENQ, or unreadable is passed over: it may be a late answer
//!   to an earlier query. The caller logs it.
//! - The exchange ends [`Limits::timeout_ms`] after its last packet without an answer. Silence never sends the query
//!   again: reference 6.5 says to check the address and port first.

use crate::packet::{ChecksumBasis, Datagram, Packet, Query, decode_datagram};
use crate::value::Reading;

/// How long an exchange waits and how often it sends. The values are `align_core::config::RuidaLink`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
  /// An exchange with no answer this long after its last packet has failed.
  pub timeout_ms: u32,
  /// The most packets one exchange sends while the controller answers NAK. Zero is taken as one.
  pub tries: u8
}

/// What the caller does next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
  /// Nothing: wait for a datagram or the deadline.
  Wait,
  /// Send this packet, the query again after a NAK.
  Send(Packet),
  /// The exchange is over.
  Done(Outcome)
}

/// How an exchange ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome {
  /// The query it sent.
  pub query: Query,
  /// Its reading, or why there is none.
  pub result: Result<Reading, Failure>,
  /// The controller's checksum basis, if a packet was answered or acknowledged: the basis to send the next query with.
  pub basis: Option<ChecksumBasis>,
  /// How many packets it sent.
  pub sends: u8
}

/// Why an exchange ended without a reading.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
  /// No answer at the query's address within the timeout of the last packet.
  TimedOut {
    /// Whether the controller acknowledged the last packet.
    acked: bool
  },
  /// Every packet was answered NAK.
  Nak,
  /// The controller answered ERR.
  Refused
}

impl Failure {
  /// The failure as the log names it.
  pub const fn name(self) -> &'static str {
    match self {
      Failure::TimedOut { acked: false } => "timed_out",
      Failure::TimedOut { acked: true } => "timed_out_after_ack",
      Failure::Nak => "nak",
      Failure::Refused => "refused"
    }
  }
}

/// One query in flight.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Exchange {
  query: Query,
  /// The basis of the last packet sent.
  basis: ChecksumBasis,
  /// Whether `basis` is the controller's, found by an earlier exchange.
  basis_known: bool,
  limits: Limits,
  sends: u8,
  /// Whether the controller acknowledged the last packet.
  acked: bool,
  deadline_ms: u64
}

impl Exchange {
  /// Start `query`: the exchange, and the packet to send now. `known` is the controller's checksum basis if an earlier
  /// exchange found it; without it the swizzled sum goes first (reference 6.5 step 2).
  pub fn start(query: Query, known: Option<ChecksumBasis>, limits: Limits, now_ms: u64) -> (Self, Packet) {
    let basis = known.unwrap_or(ChecksumBasis::Swizzled);
    let exchange = Self {
      query,
      basis,
      basis_known: known.is_some(),
      limits,
      sends: 1,
      acked: false,
      deadline_ms: now_ms.saturating_add(u64::from(limits.timeout_ms))
    };
    (exchange, query.packet(basis))
  }

  /// The query in flight.
  pub const fn query(&self) -> Query {
    self.query
  }

  /// How many packets it has sent, the first included.
  pub const fn sends(&self) -> u8 {
    self.sends
  }

  /// When the exchange fails if nothing answers it: the time to wait for a datagram until.
  pub const fn deadline_ms(&self) -> u64 {
    self.deadline_ms
  }

  /// A datagram from the controller arrived at `now_ms`.
  pub fn on_datagram(&mut self, wire: &[u8], now_ms: u64) -> Step {
    match decode_datagram(wire) {
      Ok(Datagram::Answer { address, value, .. }) if address == self.query.address() => {
        self.done(Ok(Reading::of(self.query, value)), Some(self.basis))
      }
      Ok(Datagram::Ack) => {
        self.acked = true;
        self.on_time(now_ms)
      }
      Ok(Datagram::Nak) => self.on_nak(now_ms),
      Ok(Datagram::Err) => self.done(Err(Failure::Refused), self.confirmed()),
      Ok(Datagram::Enq | Datagram::Answer { .. }) | Err(_) => self.on_time(now_ms)
    }
  }

  /// No datagram arrived by `now_ms`: the exchange fails if its deadline has passed.
  pub fn on_time(&self, now_ms: u64) -> Step {
    if now_ms >= self.deadline_ms {
      self.done(Err(Failure::TimedOut { acked: self.acked }), self.confirmed())
    } else {
      Step::Wait
    }
  }

  /// The packet again, summed the other way while the basis is not known, unless the tries are spent.
  fn on_nak(&mut self, now_ms: u64) -> Step {
    if self.sends >= self.limits.tries.max(1) {
      return self.done(Err(Failure::Nak), None);
    }
    if !self.basis_known {
      self.basis = self.basis.other();
    }
    self.sends = self.sends.saturating_add(1);
    self.acked = false;
    self.deadline_ms = now_ms.saturating_add(u64::from(self.limits.timeout_ms));
    Step::Send(self.query.packet(self.basis))
  }

  /// The basis of the last packet, if the controller acknowledged it.
  fn confirmed(&self) -> Option<ChecksumBasis> {
    self.acked.then_some(self.basis)
  }

  fn done(&self, result: Result<Reading, Failure>, basis: Option<ChecksumBasis>) -> Step {
    Step::Done(Outcome {
      query: self.query,
      result,
      basis,
      sends: self.sends
    })
  }
}

#[cfg(test)]
mod tests {
  extern crate std;

  use std::vec::Vec;

  use proptest::prelude::*;

  use super::*;
  use crate::swizzle::{MAGIC_644XG, swizzle};
  use crate::value::{MachineStatus, encode_i35, encode_u35};

  const LIMITS: Limits = Limits {
    timeout_ms: 1_000,
    tries: 2
  };

  /// A datagram as the controller sends it.
  fn wire(plain: &[u8]) -> Vec<u8> {
    plain.iter().map(|&byte| swizzle(byte, MAGIC_644XG)).collect()
  }

  fn ack() -> Vec<u8> {
    wire(&[0xCC])
  }

  fn nak() -> Vec<u8> {
    wire(&[0xCF])
  }

  fn err() -> Vec<u8> {
    wire(&[0xCD])
  }

  /// The answer at `address` carrying `value`.
  fn answer(address: [u8; 2], value: [u8; 5]) -> Vec<u8> {
    let mut plain = std::vec![0xDA, 0x01, address[0], address[1]];
    plain.extend(value);
    wire(&plain)
  }

  fn done(step: Step) -> Outcome {
    match step {
      Step::Done(outcome) => outcome,
      other => panic!("expected the exchange to be over, got {other:?}")
    }
  }

  fn sent(step: Step) -> Packet {
    match step {
      Step::Send(packet) => packet,
      other => panic!("expected a packet, got {other:?}")
    }
  }

  #[test]
  fn the_first_query_is_summed_swizzled_and_later_ones_at_the_basis_found() {
    let (_, packet) = Exchange::start(Query::Status, None, LIMITS, 0);
    assert_eq!(packet, Query::Status.packet(ChecksumBasis::Swizzled));
    let (_, packet) = Exchange::start(Query::Status, Some(ChecksumBasis::Plain), LIMITS, 0);
    assert_eq!(packet, Query::Status.packet(ChecksumBasis::Plain));
  }

  #[test]
  fn an_ack_then_the_answer_at_the_query_s_address_gives_its_reading_and_the_basis() {
    let (mut exchange, _) = Exchange::start(Query::Status, None, LIMITS, 100);
    assert_eq!(exchange.query(), Query::Status);
    assert_eq!(exchange.on_datagram(&ack(), 110), Step::Wait);
    let outcome = done(exchange.on_datagram(&answer([0x04, 0x00], encode_u35(0x0100_0001).unwrap()), 120));
    assert_eq!(
      outcome,
      Outcome {
        query: Query::Status,
        result: Ok(Reading::Status(MachineStatus::new(0x0100_0001))),
        basis: Some(ChecksumBasis::Swizzled),
        sends: 1
      }
    );
  }

  #[test]
  fn an_answer_ends_the_exchange_without_its_ack_and_after_the_deadline() {
    let (mut exchange, _) = Exchange::start(Query::PositionY, Some(ChecksumBasis::Plain), LIMITS, 0);
    let outcome = done(exchange.on_datagram(&answer([0x04, 0x31], encode_i35(-2_500).unwrap()), 5_000));
    assert_eq!(outcome.result, Ok(Reading::PositionY { y_um: -2_500 }));
    assert_eq!(outcome.basis, Some(ChecksumBasis::Plain));
  }

  #[test]
  fn a_nak_while_the_basis_is_unknown_sends_the_query_again_summed_plain() {
    let (mut exchange, _) = Exchange::start(Query::CardId, None, LIMITS, 0);
    assert_eq!(exchange.sends(), 1);
    let again = sent(exchange.on_datagram(&nak(), 10));
    assert_eq!(again, Query::CardId.packet(ChecksumBasis::Plain));
    assert_eq!(exchange.sends(), 2);
    assert_eq!(exchange.on_datagram(&ack(), 20), Step::Wait);
    let outcome = done(exchange.on_datagram(&answer([0x05, 0x7E], encode_u35(0x6500_6500).unwrap()), 30));
    assert_eq!(outcome.result, Ok(Reading::CardId { id: 0x6500_6500 }));
    assert_eq!(outcome.basis, Some(ChecksumBasis::Plain));
    assert_eq!(outcome.sends, 2);
  }

  #[test]
  fn a_nak_once_the_basis_is_known_sends_the_same_packet_again() {
    let (mut exchange, first) = Exchange::start(Query::PositionX, Some(ChecksumBasis::Swizzled), LIMITS, 0);
    assert_eq!(sent(exchange.on_datagram(&nak(), 10)), first);
  }

  #[test]
  fn a_nak_after_the_last_try_ends_the_exchange_without_a_basis() {
    let (mut exchange, _) = Exchange::start(Query::CardId, None, LIMITS, 0);
    sent(exchange.on_datagram(&nak(), 10));
    let outcome = done(exchange.on_datagram(&nak(), 20));
    assert_eq!(outcome.result, Err(Failure::Nak));
    assert_eq!(outcome.basis, None);
    assert_eq!(outcome.sends, 2);
  }

  #[test]
  fn three_tries_alternate_the_bases_while_the_basis_is_unknown() {
    let limits = Limits { tries: 3, ..LIMITS };
    let (mut exchange, _) = Exchange::start(Query::CardId, None, limits, 0);
    assert_eq!(sent(exchange.on_datagram(&nak(), 1)).basis(), ChecksumBasis::Plain);
    assert_eq!(sent(exchange.on_datagram(&nak(), 2)).basis(), ChecksumBasis::Swizzled);
    assert_eq!(done(exchange.on_datagram(&nak(), 3)).sends, 3);
  }

  #[test]
  fn no_tries_are_taken_as_one() {
    let (mut exchange, _) = Exchange::start(Query::CardId, None, Limits { tries: 0, ..LIMITS }, 0);
    assert_eq!(done(exchange.on_datagram(&nak(), 1)).result, Err(Failure::Nak));
  }

  #[test]
  fn an_err_ends_the_exchange_with_the_basis_only_if_the_packet_was_acked() {
    let (mut exchange, _) = Exchange::start(Query::MainboardVersion, None, LIMITS, 0);
    let outcome = done(exchange.on_datagram(&err(), 10));
    assert_eq!(outcome.result, Err(Failure::Refused));
    assert_eq!(outcome.basis, None);
    let (mut exchange, _) = Exchange::start(Query::MainboardVersion, None, LIMITS, 0);
    exchange.on_datagram(&ack(), 5);
    assert_eq!(
      done(exchange.on_datagram(&err(), 10)).basis,
      Some(ChecksumBasis::Swizzled)
    );
  }

  #[test]
  fn the_exchange_fails_at_its_deadline_and_not_before() {
    let (exchange, _) = Exchange::start(Query::Status, None, LIMITS, 200);
    assert_eq!(exchange.deadline_ms(), 1_200);
    assert_eq!(exchange.on_time(1_199), Step::Wait);
    let outcome = done(exchange.on_time(1_200));
    assert_eq!(outcome.result, Err(Failure::TimedOut { acked: false }));
    assert_eq!(outcome.basis, None);
  }

  #[test]
  fn an_acked_query_that_times_out_still_tells_the_basis() {
    let (mut exchange, _) = Exchange::start(Query::Status, None, LIMITS, 0);
    assert_eq!(exchange.on_datagram(&ack(), 999), Step::Wait);
    let outcome = done(exchange.on_datagram(&ack(), 1_000));
    assert_eq!(outcome.result, Err(Failure::TimedOut { acked: true }));
    assert_eq!(outcome.basis, Some(ChecksumBasis::Swizzled));
  }

  #[test]
  fn a_resend_restarts_the_deadline_and_forgets_the_ack() {
    let (mut exchange, _) = Exchange::start(Query::Status, None, LIMITS, 0);
    exchange.on_datagram(&ack(), 100);
    sent(exchange.on_datagram(&nak(), 900));
    assert_eq!(exchange.deadline_ms(), 1_900);
    assert_eq!(exchange.on_time(1_899), Step::Wait);
    assert_eq!(
      done(exchange.on_time(1_900)).result,
      Err(Failure::TimedOut { acked: false })
    );
  }

  #[test]
  fn strays_and_enquiries_are_passed_over_until_the_deadline() {
    let (mut exchange, _) = Exchange::start(Query::PositionX, None, LIMITS, 0);
    let strays = [
      answer([0x04, 0x31], [0; 5]),
      wire(&[0xCE]),
      wire(&[0xD9, 0x00]),
      std::vec![],
      wire(&[0xDA, 0x01, 0x04, 0x21, 0x80, 0, 0, 0, 0])
    ];
    for stray in &strays {
      assert_eq!(exchange.on_datagram(stray, 999), Step::Wait);
    }
    let outcome = done(exchange.on_datagram(&strays[0], 1_000));
    assert_eq!(outcome.result, Err(Failure::TimedOut { acked: false }));
    assert_eq!(outcome.sends, 1);
  }

  #[test]
  fn the_failures_are_named_for_the_log() {
    assert_eq!(Failure::TimedOut { acked: false }.name(), "timed_out");
    assert_eq!(Failure::TimedOut { acked: true }.name(), "timed_out_after_ack");
    assert_eq!(Failure::Nak.name(), "nak");
    assert_eq!(Failure::Refused.name(), "refused");
  }

  /// A datagram the controller, or anything else on the port, might send.
  fn any_datagram() -> impl Strategy<Value = Vec<u8>> {
    prop_oneof![
      Just(ack()),
      Just(nak()),
      Just(err()),
      (any::<u8>(), any::<u8>(), prop::array::uniform5(0..0x80_u8))
        .prop_map(|(high, low, value)| answer([high & 0x7F, low & 0x7F], value)),
      prop::collection::vec(any::<u8>(), 0..16)
    ]
  }

  proptest! {
    #[test]
    fn whatever_arrives_the_exchange_sends_only_its_own_query_and_at_most_its_tries(
      query in prop::sample::select(Query::ALL.to_vec()),
      known in prop::option::of(prop::sample::select(std::vec![ChecksumBasis::Swizzled, ChecksumBasis::Plain])),
      tries in 0..5_u8,
      arrivals in prop::collection::vec((any_datagram(), 0..400_u64), 0..20)
    ) {
      let limits = Limits { tries, ..LIMITS };
      let (mut exchange, first) = Exchange::start(query, known, limits, 0);
      let mut packets = std::vec![first];
      let mut now_ms = 0;
      for (datagram, after_ms) in arrivals {
        now_ms += after_ms;
        match exchange.on_datagram(&datagram, now_ms) {
          Step::Wait => {}
          Step::Send(packet) => packets.push(packet),
          Step::Done(outcome) => {
            prop_assert_eq!(outcome.query, query);
            prop_assert_eq!(usize::from(outcome.sends), packets.len());
            break;
          }
        }
      }
      prop_assert!(packets.len() <= usize::from(tries.max(1)));
      for packet in packets {
        prop_assert_eq!(packet.query(), query);
        prop_assert_eq!(packet, query.packet(packet.basis()));
        if let Some(basis) = known {
          prop_assert_eq!(packet.basis(), basis);
        }
      }
    }
  }
}
