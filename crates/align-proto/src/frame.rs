//! The 11-bit identifier layout: message class, node address, and channel.
//!
//! Reference 5.4: `[10:7] class | [6:3] node_id | [2:0] axis`. A lower identifier wins arbitration, so the class
//! sets the priority: an emergency stop (class 0) beats everything else on the bus.
//!
//! Frames and identifiers are the `embedded-can` types, which the esp-hal TWAI driver implements, so the firmware
//! hands its own frames to [`crate::Envelope`] without converting them.

use embedded_can::StandardId;

const CLASS_SHIFT: u16 = 7;
const NODE_SHIFT: u16 = 3;
const NODE_MASK: u16 = 0xF;
const AXIS_MASK: u16 = 0x7;

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

/// The message class, the 4-bit priority field of the identifier (reference 5.4; classes 0xA to 0xD from
/// `docs/SENSOR_PLAN.md` section 3).
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
  /// The laser fired, coordinator to every node.
  /// Lowest priority of all: a late SHOT costs nothing, because the sensor keeps its samples.
  Shot = 0xA,
  /// Send a reading again, coordinator to sensor.
  SensorResend = 0xB,
  /// A commissioning command, coordinator to sensor.
  SensorCmd = 0xC,
  /// The answer to a commissioning command, sensor to coordinator.
  SensorCmdReply = 0xD
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
      0xA => Self::Shot,
      0xB => Self::SensorResend,
      0xC => Self::SensorCmd,
      0xD => Self::SensorCmdReply,
      _ => return None
    })
  }

  /// The payload length every message of this class has (the DLC column of reference 5.4; HEARTBEAT is 7 since
  /// protocol version 2, which added its `proto_version` byte).
  pub const fn data_len(self) -> u8 {
    match self {
      Self::Estop | Self::Stop | Self::SensorResend => 1,
      Self::Inhibit => 2,
      Self::Fault | Self::Shot | Self::SensorCmd => 4,
      Self::Heartbeat | Self::MoveRel => 7,
      Self::MoveDone | Self::Status | Self::Sensor | Self::Config | Self::SensorCmdReply => 8
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
  pub channel: Channel
}

/// The identifier for a class, a node, and a channel.
pub const fn compose_id(class: Class, node: NodeId, channel: Channel) -> StandardId {
  let raw = ((class as u16) << CLASS_SHIFT) | ((node.0 as u16) << NODE_SHIFT) | channel.0 as u16;
  // The largest class is 0xD, so `raw` is at most 0x6FF and always fits. Were it ever not to, the fallback 0x7FF
  // names class 0xF, which every receiver rejects as undefined.
  match StandardId::new(raw) {
    Some(id) => id,
    None => StandardId::MAX
  }
}

/// The fields of an 11-bit identifier.
pub fn split_id(id: StandardId) -> IdFields {
  let raw = id.as_raw();
  IdFields {
    class_bits: ((raw >> CLASS_SHIFT) & 0xF) as u8,
    node: NodeId(((raw >> NODE_SHIFT) & NODE_MASK) as u8),
    channel: Channel((raw & AXIS_MASK) as u8)
  }
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;

  #[test]
  fn the_identifier_packs_class_node_and_axis_as_reference_5_4_lays_them_out() {
    let id = compose_id(Class::MoveRel, NodeId::MIRROR_2, Channel::new(1).unwrap());
    assert_eq!(id.as_raw(), (0x3 << 7) | (2 << 3) | 1);
    assert_eq!(id.as_raw(), 0x191);
  }

  #[test]
  fn an_emergency_stop_wins_arbitration_against_every_other_message() {
    let estop = compose_id(Class::Estop, NodeId::new(15).unwrap(), Channel::new(7).unwrap());
    for bits in 1..=0xD {
      let class = Class::from_bits(bits).unwrap();
      assert!(estop < compose_id(class, NodeId::COORDINATOR, Channel::ZERO));
    }
  }

  #[test]
  fn the_classes_above_0xd_are_not_defined() {
    for bits in 0xE..=0xF {
      assert_eq!(Class::from_bits(bits), None);
    }
  }

  #[test]
  fn the_payload_lengths_match_reference_5_4_and_the_sensor_plan() {
    // Reference 5.4, HEARTBEAT lengthened by one byte, then SHOT, SENSOR_RESEND, SENSOR_CMD, and its reply.
    let expected = [1, 2, 4, 7, 1, 8, 8, 8, 7, 8, 4, 1, 4, 8];
    for (bits, len) in (0..=0xD).zip(expected) {
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

  proptest! {
    #[test]
    fn splitting_an_identifier_recovers_its_fields(class in 0_u8..=0xD, node in 0_u8..=15, channel in 0_u8..=7) {
      let class = Class::from_bits(class).unwrap();
      let node = NodeId::new(node).unwrap();
      let channel = Channel::new(channel).unwrap();
      let fields = split_id(compose_id(class, node, channel));
      prop_assert_eq!(fields, IdFields { class_bits: class as u8, node, channel });
    }
  }
}
