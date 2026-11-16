//! The raw CAN frame and the 11-bit identifier layout.
//!
//! Reference 5.4: `[10:7] class | [6:3] node_id | [2:0] axis`. A lower identifier wins arbitration, so the class
//! sets the priority: an emergency stop (class 0) beats everything else on the bus.
//!
//! Only standard (11-bit) data frames carry this protocol. The firmware drops extended and remote frames before
//! they reach [`crate::Envelope::decode`]; this crate never sees a HAL frame type.

/// The largest payload of a classic CAN frame, in bytes.
pub const MAX_DATA_LEN: usize = 8;

/// The largest standard (11-bit) identifier.
pub const MAX_STANDARD_ID: u16 = 0x7FF;

const CLASS_SHIFT: u16 = 7;
const NODE_SHIFT: u16 = 3;
const NODE_MASK: u16 = 0xF;
const AXIS_MASK: u16 = 0x7;

/// Why a raw frame could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum FrameError {
  /// The identifier needs more than 11 bits.
  #[error("CAN identifier {id:#x} does not fit in 11 bits")]
  IdOutOfRange {
    /// The identifier given.
    id: u16,
  },
  /// The payload is longer than a classic CAN frame allows.
  #[error("CAN payload of {len} bytes is longer than the 8 a classic frame carries")]
  DataTooLong {
    /// The payload length given, in bytes.
    len: usize,
  },
}

/// A standard CAN data frame: an 11-bit identifier and up to eight bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Frame {
  id: u16,
  len: u8,
  data: [u8; MAX_DATA_LEN],
}

impl Frame {
  /// A frame from an identifier and a payload, as the bus delivered them.
  ///
  /// # Errors
  ///
  /// [`FrameError::IdOutOfRange`] above 11 bits, [`FrameError::DataTooLong`] above eight bytes.
  pub fn new(id: u16, payload: &[u8]) -> Result<Self, FrameError> {
    if id > MAX_STANDARD_ID {
      return Err(FrameError::IdOutOfRange { id });
    }
    let len = u8::try_from(payload.len())
      .ok()
      .filter(|&len| usize::from(len) <= MAX_DATA_LEN)
      .ok_or(FrameError::DataTooLong { len: payload.len() })?;
    let mut data = [0; MAX_DATA_LEN];
    for (slot, byte) in data.iter_mut().zip(payload) {
      *slot = *byte;
    }
    Ok(Self { id, len, data })
  }

  /// The 11-bit identifier.
  pub const fn id(&self) -> u16 {
    self.id
  }

  /// A frame the encoder built. The identifier is masked to 11 bits and the length capped at eight, limits the
  /// encoder never reaches.
  pub(crate) fn from_encoder(id: u16, data: [u8; MAX_DATA_LEN], len: u8) -> Self {
    Self {
      id: id & MAX_STANDARD_ID,
      len: len.min(8),
      data,
    }
  }

  /// The payload, `len` bytes long (the DLC).
  pub fn data(&self) -> &[u8] {
    self.data.get(..usize::from(self.len)).unwrap_or(&[])
  }
}

/// A node address, the 4-bit `node_id` field of the identifier.
///
/// The field names the node at the far end from the coordinator: the destination of a command, the source of a
/// report. A message to or from every node (an emergency stop, the coordinator heartbeat) carries
/// [`NodeId::COORDINATOR`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(u8);

impl NodeId {
  /// The coordinator, and the address of messages to or from every node.
  pub const COORDINATOR: Self = Self(0);
  /// The motor node at mirror 1 (hand-off 3.1: "node 1 at mirror 1").
  pub const MIRROR_1: Self = Self(1);
  /// The motor node at mirror 2 (hand-off 3.1: "node 2 at mirror 2").
  pub const MIRROR_2: Self = Self(2);
  /// The quadrant sensor node at the head.
  pub const SENSOR: Self = Self(3);
  /// The largest address the 4-bit field holds.
  pub const MAX: u8 = 15;

  /// A node address, if it fits in four bits.
  pub const fn new(value: u8) -> Option<Self> {
    if value <= Self::MAX { Some(Self(value)) } else { None }
  }

  /// The address as a number.
  pub const fn get(self) -> u8 {
    self.0
  }
}

/// A stepper channel on a node, the 3-bit `axis` field of the identifier and the `axis` byte of a payload.
///
/// This is the channel on one node board (three per board, two populated), not the solver axis of
/// `align_core::axis`: which channel turns which screw is wiring, and the coordinator maps one to the other.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Channel(u8);

impl Channel {
  /// Channel 0, and the axis field of a message that names no channel.
  pub const ZERO: Self = Self(0);
  /// The largest channel number the 3-bit field holds.
  pub const MAX: u8 = 7;

  /// A channel, if it fits in three bits.
  pub const fn new(value: u8) -> Option<Self> {
    if value <= Self::MAX { Some(Self(value)) } else { None }
  }

  /// The channel as a number.
  pub const fn get(self) -> u8 {
    self.0
  }
}

/// The message class, the 4-bit priority field of the identifier (reference 5.4).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Class {
  /// Emergency stop, coordinator to every node.
  Estop = 0x0,
  /// Motion inhibited or allowed, coordinator to every node.
  Inhibit = 0x1,
  /// A fault, node to coordinator.
  Fault = 0x2,
  /// A relative move, coordinator to node.
  MoveRel = 0x3,
  /// Stop the move in progress, coordinator to node.
  Stop = 0x4,
  /// A move ended, node to coordinator.
  MoveDone = 0x5,
  /// Axis status, node to coordinator.
  Status = 0x6,
  /// A beam reading, sensor node to coordinator.
  Sensor = 0x7,
  /// Liveness, in both directions.
  Heartbeat = 0x8,
  /// A configuration value, coordinator to node.
  Config = 0x9,
}

impl Class {
  /// The class with this number, if one is defined.
  pub const fn from_bits(bits: u8) -> Option<Self> {
    Some(match bits {
      0x0 => Self::Estop,
      0x1 => Self::Inhibit,
      0x2 => Self::Fault,
      0x3 => Self::MoveRel,
      0x4 => Self::Stop,
      0x5 => Self::MoveDone,
      0x6 => Self::Status,
      0x7 => Self::Sensor,
      0x8 => Self::Heartbeat,
      0x9 => Self::Config,
      _ => return None,
    })
  }

  /// The payload length every message of this class has (the DLC column of reference 5.4).
  pub const fn data_len(self) -> u8 {
    match self {
      Self::Estop | Self::Stop => 1,
      Self::Inhibit => 2,
      Self::Fault => 4,
      Self::Heartbeat => 6,
      Self::MoveRel => 7,
      Self::MoveDone | Self::Status | Self::Sensor | Self::Config => 8,
    }
  }
}

/// The three fields of an identifier, the class still a raw number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdFields {
  /// The 4-bit class number, which may name no defined class.
  pub class_bits: u8,
  /// The node address.
  pub node: NodeId,
  /// The channel.
  pub channel: Channel,
}

/// The identifier for a class, a node, and a channel.
pub const fn compose_id(class: Class, node: NodeId, channel: Channel) -> u16 {
  ((class as u16) << CLASS_SHIFT) | ((node.0 as u16) << NODE_SHIFT) | channel.0 as u16
}

/// The fields of an 11-bit identifier. Bits above the eleventh are ignored; [`Frame::new`] never stores them.
pub const fn split_id(id: u16) -> IdFields {
  IdFields {
    class_bits: ((id >> CLASS_SHIFT) & 0xF) as u8,
    node: NodeId(((id >> NODE_SHIFT) & NODE_MASK) as u8),
    channel: Channel((id & AXIS_MASK) as u8),
  }
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;

  #[test]
  fn the_identifier_packs_class_node_and_axis_as_reference_5_4_lays_them_out() {
    let id = compose_id(Class::MoveRel, NodeId::MIRROR_2, Channel::new(1).unwrap());
    assert_eq!(id, (0x3 << 7) | (2 << 3) | 1);
    assert_eq!(id, 0x191);
  }

  #[test]
  fn an_emergency_stop_wins_arbitration_against_every_other_message() {
    let estop = compose_id(Class::Estop, NodeId::new(15).unwrap(), Channel::new(7).unwrap());
    for bits in 1..=9 {
      let class = Class::from_bits(bits).unwrap();
      assert!(estop < compose_id(class, NodeId::COORDINATOR, Channel::new(0).unwrap()));
    }
  }

  #[test]
  fn the_classes_above_nine_are_not_defined() {
    for bits in 0xA..=0xF {
      assert_eq!(Class::from_bits(bits), None);
    }
  }

  #[test]
  fn the_payload_lengths_match_the_dlc_column_of_reference_5_4() {
    let expected = [1, 2, 4, 7, 1, 8, 8, 8, 6, 8];
    for (bits, len) in (0..=9).zip(expected) {
      assert_eq!(Class::from_bits(bits).unwrap().data_len(), len);
    }
  }

  #[test]
  fn node_addresses_and_channels_stop_at_their_field_widths() {
    assert_eq!(NodeId::new(15).map(NodeId::get), Some(15));
    assert_eq!(NodeId::new(16), None);
    assert_eq!(Channel::new(7).map(Channel::get), Some(7));
    assert_eq!(Channel::new(8), None);
  }

  #[test]
  fn a_frame_rejects_an_identifier_above_11_bits() {
    assert_eq!(Frame::new(0x800, &[]), Err(FrameError::IdOutOfRange { id: 0x800 }));
    assert!(Frame::new(0x7FF, &[]).is_ok());
  }

  #[test]
  fn a_frame_rejects_a_payload_above_eight_bytes() {
    assert_eq!(Frame::new(0, &[0; 9]), Err(FrameError::DataTooLong { len: 9 }));
    assert_eq!(Frame::new(0, &[0; 300]), Err(FrameError::DataTooLong { len: 300 }));
  }

  #[test]
  fn a_frame_returns_exactly_the_payload_it_was_given() {
    let frame = Frame::new(0x123, &[1, 2, 3]).unwrap();
    assert_eq!(frame.id(), 0x123);
    assert_eq!(frame.data(), &[1, 2, 3]);
  }

  proptest! {
    #[test]
    fn splitting_an_identifier_recovers_its_fields(class in 0_u8..=9, node in 0_u8..=15, channel in 0_u8..=7) {
      let class = Class::from_bits(class).unwrap();
      let node = NodeId::new(node).unwrap();
      let channel = Channel::new(channel).unwrap();
      let fields = split_id(compose_id(class, node, channel));
      prop_assert_eq!(fields, IdFields { class_bits: class as u8, node, channel });
    }

    #[test]
    fn every_composed_identifier_fits_in_11_bits(class in 0_u8..=9, node in 0_u8..=15, channel in 0_u8..=7) {
      let id = compose_id(
        Class::from_bits(class).unwrap(),
        NodeId::new(node).unwrap(),
        Channel::new(channel).unwrap(),
      );
      prop_assert!(id <= MAX_STANDARD_ID);
    }
  }
}
