//! What this system sends to the controller, and what the controller's datagrams say (reference 6.1, 6.3, 6.4).
//!
//! A packet is a two-byte checksum, high byte first, then the swizzled command. Which bytes the checksum sums is the
//! one fact of the format the sources disagree on (reference 6.0), so [`ChecksumBasis`] names both. The controller
//! answers each datagram on its own: a one-byte ACK, NAK, ERR, or ENQ, and a read's answer in a datagram of its own
//! with no checksum in front (MeerK40t `udp_connection.py` unswizzles the whole datagram).

use crate::swizzle::{MAGIC_644XG, swizzle, unswizzle};

/// VERIFIED (reference 6.1: EduTech Wiki, MeerK40t): the controller listens on this UDP port.
pub const CONTROLLER_PORT: u16 = 50200;

/// VERIFIED (reference 6.1; MeerK40t `udp_connection.py` binds it): the controller's answers arrive on this port.
/// Only one program on a computer can bind it, so the probe cannot run beside LightBurn (reference 6.5 step 1).
pub const REPLY_PORT: u16 = 40200;

/// The length of every packet this crate builds: the checksum and a four-byte query.
pub const PACKET_LEN: usize = 6;

/// The command byte of a memory read and of its answer (reference 6.4).
const GET: u8 = 0xDA;
/// The byte after [`GET`] that asks.
const GET_ASK: u8 = 0x00;
/// The byte after [`GET`] that answers.
const GET_ANSWER: u8 = 0x01;
/// The one-byte answers, unswizzled (reference 6.1).
const ACK: u8 = 0xCC;
const ERR: u8 = 0xCD;
const ENQ: u8 = 0xCE;
const NAK: u8 = 0xCF;
/// Bytes from here up start commands, so no data byte reaches it (reference 6.3).
const FIRST_COMMAND_BYTE: u8 = 0x80;
/// The offset of an answer's first value: after `DA 01` and the two-byte address.
const FIRST_VALUE_AT: usize = 4;
/// The length of one value (reference 6.3).
const VALUE_LEN: usize = 5;

/// Which bytes the checksum in front of a packet sums (reference 6.0: the sources disagree).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChecksumBasis {
  /// The swizzled bytes, as sent. MeerK40t's `udp_connection.py` sums these, jnweiger's `protocol.md` says "sum of
  /// scrambled message bytes", and reference 6.5 tries this first.
  Swizzled,
  /// The bytes before swizzling.
  Plain
}

impl ChecksumBasis {
  /// The other basis: the one to try after a NAK while the controller's is not known (reference 6.5 step 3).
  #[must_use]
  pub const fn other(self) -> Self {
    match self {
      ChecksumBasis::Swizzled => ChecksumBasis::Plain,
      ChecksumBasis::Plain => ChecksumBasis::Swizzled
    }
  }

  /// The basis as the log names it.
  pub const fn name(self) -> &'static str {
    match self {
      ChecksumBasis::Swizzled => "swizzled",
      ChecksumBasis::Plain => "plain"
    }
  }

  /// The checksum of a command at this basis, given it before and after swizzling: the low 16 bits of the byte sum
  /// (reference 6.1). A simulated controller checks a packet with it.
  pub fn checksum(self, plain: &[u8], wire: &[u8]) -> u16 {
    let bytes = match self {
      ChecksumBasis::Swizzled => wire,
      ChecksumBasis::Plain => plain
    };
    bytes
      .iter()
      .fold(0, |sum: u16, &byte| sum.wrapping_add(u16::from(byte)))
  }
}

/// The five memory reads this system may send (reference 6.4, MeerK40t `rdjob.py`): `DA 00` and an address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Query {
  /// `05 7E`: the card ID (reference 6.0: do not hard-code the expected value).
  CardId,
  /// `05 7F`: the mainboard version (EduTech Wiki; MeerK40t does not read it).
  MainboardVersion,
  /// `04 00`: the machine status bits.
  Status,
  /// `04 21`: the current X position in micrometers.
  PositionX,
  /// `04 31`: the current Y position in micrometers.
  PositionY
}

impl Query {
  /// Every query, in the order of the first test (reference 6.5 steps 2 and 4).
  pub const ALL: [Query; 5] = [
    Query::CardId,
    Query::MainboardVersion,
    Query::Status,
    Query::PositionX,
    Query::PositionY
  ];

  /// The memory address the query reads.
  pub const fn address(self) -> [u8; 2] {
    match self {
      Query::CardId => [0x05, 0x7E],
      Query::MainboardVersion => [0x05, 0x7F],
      Query::Status => [0x04, 0x00],
      Query::PositionX => [0x04, 0x21],
      Query::PositionY => [0x04, 0x31]
    }
  }

  /// The query whose address this is, if it is one of the five.
  pub fn at(address: [u8; 2]) -> Option<Query> {
    Query::ALL.into_iter().find(|query| query.address() == address)
  }

  /// The query as the log names it.
  pub const fn name(self) -> &'static str {
    match self {
      Query::CardId => "card_id",
      Query::MainboardVersion => "mainboard_version",
      Query::Status => "status",
      Query::PositionX => "x",
      Query::PositionY => "y"
    }
  }

  /// The command before swizzling: `DA 00` and the address.
  pub const fn plain(self) -> [u8; 4] {
    let [high, low] = self.address();
    [GET, GET_ASK, high, low]
  }

  /// The packet that sends the query, swizzled at [`MAGIC_644XG`], its checksum summed at `basis`.
  pub fn packet(self, basis: ChecksumBasis) -> Packet {
    let plain = self.plain();
    let wire = plain.map(|byte| swizzle(byte, MAGIC_644XG));
    let [sum_high, sum_low] = basis.checksum(&plain, &wire).to_be_bytes();
    let [w0, w1, w2, w3] = wire;
    Packet {
      bytes: [sum_high, sum_low, w0, w1, w2, w3],
      query: self,
      basis
    }
  }
}

/// One query as it goes on the wire. Only [`Query::packet`] makes one, so only the five reads can be sent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Packet {
  bytes: [u8; PACKET_LEN],
  query: Query,
  basis: ChecksumBasis
}

impl Packet {
  /// The datagram to send to [`CONTROLLER_PORT`].
  pub const fn bytes(&self) -> &[u8; PACKET_LEN] {
    &self.bytes
  }

  /// The query the packet sends.
  pub const fn query(&self) -> Query {
    self.query
  }

  /// What its checksum sums.
  pub const fn basis(&self) -> ChecksumBasis {
    self.basis
  }
}

/// What one datagram from the controller says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Datagram {
  /// The last packet was accepted.
  Ack,
  /// The last packet should be sent again (reference 6.1).
  Nak,
  /// The last packet was refused.
  Err,
  /// An enquiry from the controller (reference 6.1 lists it without saying when it comes).
  Enq,
  /// A read's answer (`DA 01`, the address, a value). Reference 6.4 shows two values and MeerK40t reads one, so the
  /// second is kept when a whole one follows; bytes after it are ignored.
  Answer {
    /// The address it answers.
    address: [u8; 2],
    /// Its first value, five data bytes.
    value: [u8; VALUE_LEN],
    /// A second value, if the datagram is long enough to hold one.
    second: Option<[u8; VALUE_LEN]>
  }
}

/// Why a datagram from the controller says nothing this crate can read. Each carries the bytes as unswizzled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DatagramError {
  /// It was empty.
  Empty,
  /// One byte that is none of the four answers.
  UnknownByte {
    /// The byte, unswizzled.
    byte: u8
  },
  /// It starts as a read's answer but ends before the address and one whole value.
  Truncated {
    /// Its length in bytes.
    len: usize
  },
  /// A byte of a read's answer at or above 0x80, where only data may be. A wrong magic gives this.
  NotData {
    /// Where it is in the datagram.
    offset: usize,
    /// The byte, unswizzled.
    byte: u8
  },
  /// Neither a one-byte answer nor a read's answer.
  Unrecognized {
    /// Its first two bytes, unswizzled.
    start: [u8; 2],
    /// Its length in bytes.
    len: usize
  }
}

/// What a datagram from the controller says, unswizzled at [`MAGIC_644XG`].
///
/// # Errors
///
/// [`DatagramError`] when it is empty, an unknown single byte, or not a whole read's answer of data bytes.
pub fn decode_datagram(wire: &[u8]) -> Result<Datagram, DatagramError> {
  let plain_at = |offset: usize| wire.get(offset).map(|&byte| unswizzle(byte, MAGIC_644XG));
  let (Some(first), second) = (plain_at(0), plain_at(1)) else {
    return Err(DatagramError::Empty);
  };
  let Some(second) = second else {
    return match first {
      ACK => Ok(Datagram::Ack),
      NAK => Ok(Datagram::Nak),
      ERR => Ok(Datagram::Err),
      ENQ => Ok(Datagram::Enq),
      byte => Err(DatagramError::UnknownByte { byte })
    };
  };
  if [first, second] != [GET, GET_ANSWER] {
    return Err(DatagramError::Unrecognized {
      start: [first, second],
      len: wire.len()
    });
  }
  let data_at = |offset: usize| {
    let byte = plain_at(offset).ok_or(DatagramError::Truncated { len: wire.len() })?;
    if byte < FIRST_COMMAND_BYTE {
      Ok(byte)
    } else {
      Err(DatagramError::NotData { offset, byte })
    }
  };
  let value_at = |start: usize| -> Result<[u8; VALUE_LEN], DatagramError> {
    Ok([
      data_at(start)?,
      data_at(start + 1)?,
      data_at(start + 2)?,
      data_at(start + 3)?,
      data_at(start + 4)?
    ])
  };
  let second_at = FIRST_VALUE_AT + VALUE_LEN;
  Ok(Datagram::Answer {
    address: [data_at(2)?, data_at(3)?],
    value: value_at(FIRST_VALUE_AT)?,
    second: if wire.len() >= second_at + VALUE_LEN {
      Some(value_at(second_at)?)
    } else {
      None
    }
  })
}

#[cfg(test)]
mod tests {
  extern crate std;

  use std::vec;
  use std::vec::Vec;

  use super::*;

  /// The datagram the controller would send for these plain bytes.
  fn from_controller(plain: &[u8]) -> Vec<u8> {
    plain.iter().map(|&byte| swizzle(byte, MAGIC_644XG)).collect()
  }

  #[test]
  fn the_queries_read_the_addresses_of_reference_6_4() {
    assert_eq!(Query::CardId.plain(), [0xDA, 0x00, 0x05, 0x7E]);
    assert_eq!(Query::MainboardVersion.plain(), [0xDA, 0x00, 0x05, 0x7F]);
    assert_eq!(Query::Status.plain(), [0xDA, 0x00, 0x04, 0x00]);
    assert_eq!(Query::PositionX.plain(), [0xDA, 0x00, 0x04, 0x21]);
    assert_eq!(Query::PositionY.plain(), [0xDA, 0x00, 0x04, 0x31]);
  }

  #[test]
  fn each_query_is_found_by_its_own_address_and_no_other_address_is_a_query() {
    for query in Query::ALL {
      assert_eq!(Query::at(query.address()), Some(query));
    }
    assert_eq!(Query::at([0x04, 0x05]), None);
    assert_eq!(Query::at([0x00, 0x26]), None);
  }

  #[test]
  fn the_card_id_query_goes_out_swizzled_behind_its_checksum_high_byte_first() {
    // DA 00 05 7E at 0x88: DA -> 0x5B ^ 0x88 + 1 = 0xD4, 00 -> 0x89, 05 -> 0x84 ^ 0x88 + 1 = 0x0D,
    // 7E -> 0x7E ^ 0x88 + 1 = 0xF7. Their sum is 0x261; the plain bytes sum to 0x15D.
    let packet = Query::CardId.packet(ChecksumBasis::Swizzled);
    assert_eq!(packet.bytes(), &[0x02, 0x61, 0xD4, 0x89, 0x0D, 0xF7]);
    assert_eq!(packet.query(), Query::CardId);
    assert_eq!(packet.basis(), ChecksumBasis::Swizzled);
    let packet = Query::CardId.packet(ChecksumBasis::Plain);
    assert_eq!(packet.bytes(), &[0x01, 0x5D, 0xD4, 0x89, 0x0D, 0xF7]);
    assert_eq!(packet.basis(), ChecksumBasis::Plain);
  }

  #[test]
  fn every_packet_unswizzles_to_its_query_behind_the_sum_of_the_bytes_its_basis_names() {
    for query in Query::ALL {
      for basis in [ChecksumBasis::Swizzled, ChecksumBasis::Plain] {
        let bytes = query.packet(basis).bytes().to_vec();
        let wire = &bytes[2..];
        let plain: Vec<u8> = wire.iter().map(|&byte| unswizzle(byte, MAGIC_644XG)).collect();
        assert_eq!(plain, query.plain());
        let summed = match basis {
          ChecksumBasis::Swizzled => wire,
          ChecksumBasis::Plain => &plain[..]
        };
        let sum: u32 = summed.iter().map(|&byte| u32::from(byte)).sum();
        assert_eq!(u32::from(u16::from_be_bytes([bytes[0], bytes[1]])), sum);
      }
    }
  }

  #[test]
  fn the_two_bases_differ_for_every_query_so_a_nak_can_tell_them_apart() {
    for query in Query::ALL {
      assert_ne!(
        query.packet(ChecksumBasis::Swizzled).bytes()[..2],
        query.packet(ChecksumBasis::Plain).bytes()[..2]
      );
    }
  }

  #[test]
  fn a_checksum_wraps_at_16_bits() {
    let wire = [0xFF; 300];
    // 300 * 255 = 76500 = 0x12AD4, of which the low 16 bits are 0x2AD4.
    assert_eq!(ChecksumBasis::Swizzled.checksum(&[], &wire), 0x2AD4);
    assert_eq!(ChecksumBasis::Plain.checksum(&wire, &[]), 0x2AD4);
  }

  #[test]
  fn each_basis_has_the_other_for_its_retry() {
    assert_eq!(ChecksumBasis::Swizzled.other(), ChecksumBasis::Plain);
    assert_eq!(ChecksumBasis::Plain.other(), ChecksumBasis::Swizzled);
    assert_eq!(ChecksumBasis::Swizzled.name(), "swizzled");
    assert_eq!(ChecksumBasis::Plain.name(), "plain");
  }

  #[test]
  fn a_query_swizzled_at_the_644xg_magic_reads_at_the_other_documented_magics_as_other_commands() {
    // Reference 6.2 documents 0x11 (634XG) and 0x38 (RDL9635) too. A controller at either would run these bytes
    // instead of a read: each starts with something other than DA 00, and each holds a byte at or above 0x80, which
    // starts a command (reference 6.3). This is why the magic is fixed.
    let read_at = |query: Query, magic: u8| -> Vec<u8> {
      query.packet(ChecksumBasis::Swizzled).bytes()[2..]
        .iter()
        .map(|&byte| unswizzle(byte, magic))
        .collect()
    };
    let at_634xg: Vec<Vec<u8>> = Query::ALL.into_iter().map(|query| read_at(query, 0x11)).collect();
    assert_eq!(
      at_634xg,
      [
        vec![0x43, 0x99, 0x9C, 0xE7],
        vec![0x43, 0x99, 0x9C, 0xE6],
        vec![0x43, 0x99, 0x9D, 0x99],
        vec![0x43, 0x99, 0x9D, 0xB8],
        vec![0x43, 0x99, 0x9D, 0xA8]
      ]
    );
    let at_9635: Vec<Vec<u8>> = Query::ALL.into_iter().map(|query| read_at(query, 0x38)).collect();
    assert_eq!(
      at_9635,
      [
        vec![0xEB, 0x31, 0x34, 0x4F],
        vec![0xEB, 0x31, 0x34, 0x4E],
        vec![0xEB, 0x31, 0x35, 0x31],
        vec![0xEB, 0x31, 0x35, 0x10],
        vec![0xEB, 0x31, 0x35, 0x00]
      ]
    );
  }

  #[test]
  fn the_one_byte_answers_are_read_from_the_bytes_reference_6_1_gives() {
    assert_eq!(decode_datagram(&[0xC6]), Ok(Datagram::Ack));
    assert_eq!(decode_datagram(&[0x48]), Ok(Datagram::Nak));
    assert_eq!(decode_datagram(&[0x46]), Ok(Datagram::Err));
    assert_eq!(decode_datagram(&[0xC8]), Ok(Datagram::Enq));
  }

  #[test]
  fn an_empty_datagram_or_an_unknown_byte_says_nothing() {
    assert_eq!(decode_datagram(&[]), Err(DatagramError::Empty));
    // An ACK swizzled at another magic: 0xCC at 0x11 is 0x5D.
    assert_eq!(
      decode_datagram(&[swizzle(0xCC, 0x11)]),
      Err(DatagramError::UnknownByte {
        byte: unswizzle(swizzle(0xCC, 0x11), MAGIC_644XG)
      })
    );
    assert_eq!(
      decode_datagram(&from_controller(&[0xCB])),
      Err(DatagramError::UnknownByte { byte: 0xCB })
    );
  }

  #[test]
  fn an_answer_is_its_address_and_its_value() {
    let datagram = from_controller(&[0xDA, 0x01, 0x05, 0x7E, 0x06, 0x28, 0x01, 0x4A, 0x00]);
    assert_eq!(
      decode_datagram(&datagram),
      Ok(Datagram::Answer {
        address: [0x05, 0x7E],
        value: [0x06, 0x28, 0x01, 0x4A, 0x00],
        second: None
      })
    );
  }

  #[test]
  fn an_answer_keeps_a_second_whole_value_and_ignores_what_follows_it() {
    let plain = [0xDA, 0x01, 0x04, 0x21, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    let expected = Ok(Datagram::Answer {
      address: [0x04, 0x21],
      value: [1, 2, 3, 4, 5],
      second: Some([6, 7, 8, 9, 10])
    });
    assert_eq!(decode_datagram(&from_controller(&plain)), expected);
    let mut longer = plain.to_vec();
    longer.extend([0xFF, 0x11]);
    assert_eq!(decode_datagram(&from_controller(&longer)), expected);
    // Four bytes of a second value are not one.
    assert_eq!(
      decode_datagram(&from_controller(&plain[..13])),
      Ok(Datagram::Answer {
        address: [0x04, 0x21],
        value: [1, 2, 3, 4, 5],
        second: None
      })
    );
  }

  #[test]
  fn an_answer_without_a_whole_value_is_truncated() {
    for len in 2..9 {
      let plain = &[0xDA, 0x01, 0x04, 0x00, 0, 0, 0, 0, 0][..len];
      assert_eq!(
        decode_datagram(&from_controller(plain)),
        Err(DatagramError::Truncated { len }),
        "length {len}"
      );
    }
  }

  #[test]
  fn a_command_byte_inside_an_answer_is_not_data_wherever_it_is() {
    let answer = [0xDA, 0x01, 0x04, 0x00, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5];
    for offset in 2..answer.len() {
      let mut plain = answer;
      plain[offset] = 0x80;
      assert_eq!(
        decode_datagram(&from_controller(&plain)),
        Err(DatagramError::NotData { offset, byte: 0x80 }),
        "offset {offset}"
      );
    }
    let mut plain = answer;
    plain[4] = 0x7F;
    assert!(decode_datagram(&from_controller(&plain)).is_ok());
  }

  #[test]
  fn two_bytes_or_more_that_are_not_a_read_s_answer_are_unrecognized() {
    for start in [[0xDA, 0x00], [0xD9, 0x01], [0xCC, 0xCC]] {
      let datagram = from_controller(&[start[0], start[1], 0, 0]);
      assert_eq!(
        decode_datagram(&datagram),
        Err(DatagramError::Unrecognized { start, len: 4 })
      );
    }
  }

  #[test]
  fn the_ports_are_the_ones_reference_6_1_gives() {
    assert_eq!(CONTROLLER_PORT, 50200);
    assert_eq!(REPLY_PORT, 40200);
  }
}
