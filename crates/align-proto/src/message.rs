//! The fourteen messages and their fixed little-endian layouts: the ten of reference 5.4 with the field changes of
//! addendum A.8, and the sensor's four of `docs/SENSOR_PLAN.md` section 3 (protocol version 2).
//!
//! Every payload has exactly the length its class names, and the decoder rejects any other length. Where a payload
//! carries an `axis` byte, the identifier's 3-bit axis field must hold the same channel; where it carries none,
//! the field must be zero. Reserved bytes (the pads) are written as zero and ignored when read, so that a later
//! layout can use them. Fields whose meaning the documents leave open (stop and estop reasons, the inhibit source,
//! the heartbeat state) travel as raw bytes until the milestone that gives them meaning.

use embedded_can::{Frame, Id};

use crate::frame::{Channel, Class, NodeId, compose_id, split_id};
use crate::sensor::{
  CmdStatus, EDGE_AGE_UNKNOWN, SensorCmd, SensorCmdReply, SensorOp, SensorReading, SensorStatus, Shot, ShotPoint
};

/// A fault code in a FAULT message. The documents define only the two codes addendum A.8 adds; any other value
/// is carried through unchanged so that the coordinator can still report it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FaultCode(pub u8);

impl FaultCode {
  /// The beam data shows an axis that does not move when commanded (addendum A.8, 0x10).
  pub const AXIS_NOT_RESPONDING: Self = Self(0x10);
  /// The switched motor rail dropped while a move was running (addendum A.8, 0x11).
  pub const RAIL_LOST_DURING_MOVE: Self = Self(0x11);

  // The codes below are defined by milestone M4, for the reasons a node refuses a command. For a refused MOVE_REL
  // the detail is its sequence number; for a refused CONFIG, its key.

  /// The node has not restored its positions from flash yet.
  pub const NOT_READY: Self = Self(0x20);
  /// Motion is inhibited (after boot, an INHIBIT, or an ESTOP) until the coordinator allows it.
  pub const INHIBITED: Self = Self(0x21);
  /// No coordinator heartbeat within the timeout.
  pub const NO_HEARTBEAT: Self = Self(0x22);
  /// The hardware motor permit is not present.
  pub const NO_PERMIT: Self = Self(0x23);
  /// The channel has no motor on this node.
  pub const CHANNEL_UNAVAILABLE: Self = Self(0x24);
  /// The channel lacks its soft limits or direction sign: CONFIG them first.
  pub const NOT_CONFIGURED: Self = Self(0x25);
  /// The move exceeds the step or time cap of one move (addendum A.5).
  pub const MOVE_TOO_LONG: Self = Self(0x26);
  /// A different move is running.
  pub const BUSY: Self = Self(0x27);
  /// The sequence number of the last move arrived with different content.
  pub const SEQ_REUSED: Self = Self(0x28);
  /// A CONFIG value is out of range, or arrived while a move runs.
  pub const CONFIG_REJECTED: Self = Self(0x29);
  /// A coil output failed; the position is suspect.
  pub const PIN_FAULT: Self = Self(0x2A);
  /// The node is in bench mode, driven from its console, and refuses bus moves.
  pub const BENCH_MODE: Self = Self(0x2B);
  /// A move outlasted its time cap and was cut off; the steps it issued are unknown.
  pub const MOVE_TIMEOUT: Self = Self(0x2C);
  /// Writing a position record to flash failed.
  pub const PERSIST_FAILED: Self = Self(0x2D);

  // The codes below are defined by the sensor plan (`docs/SENSOR_PLAN.md` section 3.5); 0x34 applies to every node.

  /// The sensor's front end failed.
  pub const FRONT_END_FAULT: Self = Self(0x30);
  /// The sensor's sampler fell behind, or a shot's samples left the buffer before it was processed.
  pub const BUFFER_OVERRUN: Self = Self(0x31);
  /// The sensor runs on default calibration values.
  pub const CALIBRATION_MISSING: Self = Self(0x32);
  /// The sensor saw a fire with no SHOT. Evidence for the log only: the interlock does not depend on it.
  pub const UNEXPECTED_FIRE: Self = Self(0x33);
  /// The coordinator's heartbeat carries another protocol version; the detail is that version. The node refuses
  /// to act until the versions match.
  pub const PROTOCOL_MISMATCH: Self = Self(0x34);
}

/// A relative move of one channel (MOVE_REL, class 0x3).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MoveRel {
  /// Sequence number. A repeat of the last move with the same number is not executed again ([`crate::dedup`]).
  pub seq: u8,
  /// The channel to move.
  pub channel: Channel,
  /// The move, in full steps (addendum A.8), signed by direction.
  pub delta_steps: i32,
  /// The move ends in the plus direction, with the overshoot of the standard approach (flags bit 0).
  pub finish_plus: bool
}

/// How a move ended (the `result` byte of MOVE_DONE, addendum A.8).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MoveResult {
  /// Every step was issued.
  Done = 0,
  /// A STOP, an emergency stop, or a lost heartbeat ended it early.
  Stopped = 1,
  /// The switched motor rail dropped during the move: the position is no longer trusted.
  RailLost = 2,
  /// The move would have crossed a soft limit.
  SoftLimit = 3
}

impl MoveResult {
  const fn from_byte(value: u8) -> Option<Self> {
    Some(match value {
      0 => Self::Done,
      1 => Self::Stopped,
      2 => Self::RailLost,
      3 => Self::SoftLimit,
      _ => return None
    })
  }
}

/// A move has ended (MOVE_DONE, class 0x5).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MoveDone {
  /// The sequence number of the move this answers.
  pub seq: u8,
  /// The channel that moved.
  pub channel: Channel,
  /// How the move ended.
  pub result: MoveResult,
  /// The commanded position after the move, in full steps. Without position feedback this is
  /// the count of steps issued, not a measurement.
  pub pos_steps: i32
}

bitflags::bitflags! {
  /// The flag byte of a STATUS message, with the meanings of addendum A.8. All eight bits are defined.
  #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
  pub struct StatusFlags: u8 {
    /// Bit 0: a move is running.
    const MOVING = 0b0000_0001;
    /// Bit 1: the last move reached its target.
    const AT_TARGET = 0b0000_0010;
    /// Bit 2: the position is not trusted (addendum A.8; was `stalled`).
    const SUSPECT = 0b0000_0100;
    /// Bit 3: the switched motor rail is up (addendum A.8; was `nfault`).
    const RAIL_OK = 0b0000_1000;
    /// Bit 4: the position is at a soft limit.
    const SOFT_LIMIT = 0b0001_0000;
    /// Bit 5: motion is inhibited.
    const INHIBITED = 0b0010_0000;
    /// Bit 6: the position is saved to flash.
    const PERSISTED = 0b0100_0000;
    /// Bit 7: the arm pulse is running.
    const ARMED = 0b1000_0000;
  }
}

/// The state of one channel (STATUS, class 0x6).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Status {
  /// The channel reported.
  pub channel: Channel,
  /// Its flags.
  pub flags: StatusFlags,
  /// The commanded position, in full steps.
  pub pos_steps: i32,
  /// Motor current. Always 0 from a stepper node (addendum A.8); kept for a DC gearmotor node, which measures it.
  pub current_ma: u16
}

/// A configuration key (addendum A.8). The node rejects any other key.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ConfigKey {
  /// The time between full steps, in microseconds.
  StepPeriodUs = 1,
  /// The overshoot of the standard approach, in full steps.
  OvershootSteps = 2,
  /// The lower soft limit, in full steps.
  SoftMinSteps = 3,
  /// The upper soft limit, in full steps.
  SoftMaxSteps = 4,
  /// `+1` or `-1`: which way the motor turns for a plus step.
  DirSign = 5
}

impl ConfigKey {
  const fn from_byte(value: u8) -> Option<Self> {
    Some(match value {
      1 => Self::StepPeriodUs,
      2 => Self::OvershootSteps,
      3 => Self::SoftMinSteps,
      4 => Self::SoftMaxSteps,
      5 => Self::DirSign,
      _ => return None
    })
  }
}

/// One message on the bus, without its address.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Message {
  /// Stop every motor and disarm (class 0x0).
  Estop {
    /// Why. Values are not yet defined.
    reason: u8
  },
  /// Motion is inhibited, or allowed again (class 0x1).
  Inhibit {
    /// Motion is inhibited.
    inhibited: bool,
    /// What inhibited it. Values are not yet defined.
    source: u8
  },
  /// A node reports a fault (class 0x2).
  Fault {
    /// The channel the fault concerns.
    channel: Channel,
    /// What went wrong.
    code: FaultCode,
    /// Code-specific detail.
    detail: u16
  },
  /// Move one channel (class 0x3).
  MoveRel(MoveRel),
  /// Stop the move in progress (class 0x4).
  Stop {
    /// Why. Values are not yet defined.
    reason: u8
  },
  /// A move has ended (class 0x5).
  MoveDone(MoveDone),
  /// The state of one channel (class 0x6).
  Status(Status),
  /// A beam reading (class 0x7).
  Sensor(SensorReading),
  /// The sender is alive (class 0x8). Nodes stop and disarm after 300 ms without one from the coordinator.
  Heartbeat {
    /// Time since the sender booted, in milliseconds.
    uptime_ms: u32,
    /// The sender's state. Values are defined with the state machines (M4, M5).
    state: u8,
    /// The sender's firmware version.
    fw_version: u8,
    /// The sender's protocol version, [`crate::PROTOCOL_VERSION`] when it matches this crate.
    proto_version: u8
  },
  /// Set a configuration value on one channel (class 0x9).
  Config {
    /// Which value.
    key: ConfigKey,
    /// The channel it applies to.
    channel: Channel,
    /// The value, in the unit the key names.
    value: i32
  },
  /// The laser fired (class 0xA).
  Shot(Shot),
  /// Send the reading for this shot again (class 0xB).
  SensorResend {
    /// The SHOT `seq` whose reading was lost.
    seq: u8
  },
  /// A commissioning command for the sensor (class 0xC).
  SensorCmd(SensorCmd),
  /// The sensor's answer to a commissioning command (class 0xD).
  SensorCmdReply(SensorCmdReply)
}

impl Message {
  /// The class of this message.
  pub const fn class(&self) -> Class {
    match self {
      Self::Estop { .. } => Class::Estop,
      Self::Inhibit { .. } => Class::Inhibit,
      Self::Fault { .. } => Class::Fault,
      Self::MoveRel(_) => Class::MoveRel,
      Self::Stop { .. } => Class::Stop,
      Self::MoveDone(_) => Class::MoveDone,
      Self::Status(_) => Class::Status,
      Self::Sensor(_) => Class::Sensor,
      Self::Heartbeat { .. } => Class::Heartbeat,
      Self::Config { .. } => Class::Config,
      Self::Shot(_) => Class::Shot,
      Self::SensorResend { .. } => Class::SensorResend,
      Self::SensorCmd(_) => Class::SensorCmd,
      Self::SensorCmdReply(_) => Class::SensorCmdReply
    }
  }

  /// The channel this message is about, for the messages that name one.
  pub const fn channel(&self) -> Option<Channel> {
    match self {
      Self::Fault { channel, .. }
      | Self::Config { channel, .. }
      | Self::MoveRel(MoveRel { channel, .. })
      | Self::MoveDone(MoveDone { channel, .. })
      | Self::Status(Status { channel, .. }) => Some(*channel),
      Self::Estop { .. }
      | Self::Inhibit { .. }
      | Self::Stop { .. }
      | Self::Sensor(_)
      | Self::Heartbeat { .. }
      | Self::Shot(_)
      | Self::SensorResend { .. }
      | Self::SensorCmd(_)
      | Self::SensorCmdReply(_) => None
    }
  }
}

/// Why a message could not be put in a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum EncodeError {
  /// The frame type refused the identifier and payload. Every payload here is at most eight bytes, so a CAN 2.0
  /// frame type never does this; a frame type with tighter limits might.
  #[error("the frame type refused a {class:?} frame")]
  FrameRejected {
    /// The class of the message.
    class: Class
  }
}

/// Why a frame is not a valid message.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
  /// The frame has a 29-bit identifier. This protocol uses standard 11-bit identifiers only.
  #[error("CAN frame has the extended identifier {raw:#x}; the protocol uses 11-bit identifiers only")]
  ExtendedId {
    /// The 29-bit identifier.
    raw: u32
  },
  /// The frame is a remote (RTR) frame, which carries no data. The protocol does not use them.
  #[error("CAN frame {raw:#x} is a remote frame; the protocol uses data frames only")]
  RemoteFrame {
    /// The 11-bit identifier.
    raw: u16
  },
  /// The identifier names a class above 0xD.
  #[error("CAN identifier names class {class_bits:#x}, which is not defined")]
  UnknownClass {
    /// The class number.
    class_bits: u8
  },
  /// The payload length is not the one the class requires.
  #[error("{class:?} needs a {expected}-byte payload, the frame has {got}")]
  WrongLength {
    /// The class of the frame.
    class: Class,
    /// The length the class requires.
    expected: u8,
    /// The length received.
    got: usize
  },
  /// The payload's axis byte and the identifier's axis field differ.
  #[error("{class:?} names channel {payload} in its payload but channel {id} in its identifier")]
  ChannelMismatch {
    /// The class of the frame.
    class: Class,
    /// The channel in the identifier.
    id: u8,
    /// The axis byte in the payload.
    payload: u8
  },
  /// A message that names no channel has a nonzero axis field in its identifier.
  #[error("{class:?} names no channel, but its identifier's axis field is {id}")]
  UnexpectedChannel {
    /// The class of the frame.
    class: Class,
    /// The axis field of the identifier.
    id: u8
  },
  /// The `inhibited` byte of an INHIBIT is neither 0 nor 1.
  #[error("Inhibit carries {value} in its inhibited byte, which must be 0 or 1")]
  NotABool {
    /// The byte received.
    value: u8
  },
  /// A MOVE_REL sets flag bits the protocol does not define. A move is never run on a guess at what they mean.
  #[error("MoveRel sets undefined flag bits ({flags:#010b}); only bit 0 (finish_plus) is defined")]
  UndefinedMoveFlags {
    /// The flag byte received.
    flags: u8
  },
  /// The `result` byte of a MOVE_DONE is not one of the four results of addendum A.8.
  #[error("MoveDone carries result {value}, which is not defined (0 to 3 are)")]
  UnknownMoveResult {
    /// The byte received.
    value: u8
  },
  /// The key of a CONFIG is not one of the five keys of addendum A.8.
  #[error("Config names key {key}, which is not defined (1 to 5 are)")]
  UnknownConfigKey {
    /// The key received.
    key: u8
  },
  /// A SENSOR carries a result code that is not defined. The coordinator cannot judge such a reading.
  #[error("Sensor carries result code {code}, which is not defined (0 to 4, 6, 7, and 9 are)")]
  UnknownSensorResult {
    /// The low five bits of the status byte.
    code: u8
  },
  /// A SENSOR_CMD or its reply names an operation that is not defined.
  #[error("SensorCmd names operation {op}, which is not defined (1 to 6 are)")]
  UnknownSensorOp {
    /// The byte received.
    op: u8
  },
  /// A SENSOR_CMD_REPLY carries a status that is not defined.
  #[error("SensorCmdReply carries status {value}, which is not defined (0 to 3 are)")]
  UnknownCmdStatus {
    /// The byte received.
    value: u8
  }
}

const MOVE_FLAG_FINISH_PLUS: u8 = 0b0000_0001;

/// A message and the node it goes to or comes from: everything one frame carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Envelope {
  /// The node at the far end from the coordinator ([`NodeId::COORDINATOR`] for messages to or from every node).
  pub node: NodeId,
  /// The message.
  pub message: Message
}

impl Envelope {
  /// The frame that carries this message, in any `embedded-can` frame type (the esp-hal TWAI frame in firmware).
  ///
  /// # Errors
  ///
  /// [`EncodeError::FrameRejected`] if the frame type refuses the frame, which a CAN 2.0 frame type never does.
  pub fn encode<F: Frame>(&self) -> Result<F, EncodeError> {
    let class = self.message.class();
    let channel = self.message.channel().unwrap_or(Channel::ZERO);
    let id = compose_id(class, self.node, channel);
    let data: [u8; 8] = match self.message {
      Message::Estop { reason } | Message::Stop { reason } => [reason, 0, 0, 0, 0, 0, 0, 0],
      Message::Inhibit { inhibited, source } => [u8::from(inhibited), source, 0, 0, 0, 0, 0, 0],
      Message::Fault { channel, code, detail } => {
        let [d0, d1] = detail.to_le_bytes();
        [channel.get(), code.0, d0, d1, 0, 0, 0, 0]
      }
      Message::MoveRel(MoveRel {
        seq,
        channel,
        delta_steps,
        finish_plus
      }) => {
        let [d0, d1, d2, d3] = delta_steps.to_le_bytes();
        let flags = if finish_plus { MOVE_FLAG_FINISH_PLUS } else { 0 };
        [seq, channel.get(), d0, d1, d2, d3, flags, 0]
      }
      Message::MoveDone(MoveDone {
        seq,
        channel,
        result,
        pos_steps
      }) => {
        let [p0, p1, p2, p3] = pos_steps.to_le_bytes();
        [seq, channel.get(), result as u8, 0, p0, p1, p2, p3]
      }
      Message::Status(Status {
        channel,
        flags,
        pos_steps,
        current_ma
      }) => {
        let [p0, p1, p2, p3] = pos_steps.to_le_bytes();
        let [c0, c1] = current_ma.to_le_bytes();
        [channel.get(), flags.bits(), p0, p1, p2, p3, c0, c1]
      }
      Message::Sensor(SensorReading {
        seq,
        status,
        h_q15,
        v_q15,
        sum_q
      }) => {
        let [h0, h1] = h_q15.to_le_bytes();
        let [v0, v1] = v_q15.to_le_bytes();
        let [s0, s1] = sum_q.to_le_bytes();
        [seq, status.to_byte(), h0, h1, v0, v1, s0, s1]
      }
      Message::Heartbeat {
        uptime_ms,
        state,
        fw_version,
        proto_version
      } => {
        let [u0, u1, u2, u3] = uptime_ms.to_le_bytes();
        [u0, u1, u2, u3, state, fw_version, proto_version, 0]
      }
      Message::Config { key, channel, value } => {
        let [v0, v1, v2, v3] = value.to_le_bytes();
        [key as u8, channel.get(), 0, 0, v0, v1, v2, v3]
      }
      Message::Shot(Shot {
        seq,
        point,
        edge_age_10us
      }) => {
        // A known age saturates one below the "unknown" value, so that it is never read back as unknown.
        let age = edge_age_10us.map_or(EDGE_AGE_UNKNOWN, |age| age.min(EDGE_AGE_UNKNOWN - 1));
        let [a0, a1] = age.to_le_bytes();
        [seq, point.to_byte(), a0, a1, 0, 0, 0, 0]
      }
      Message::SensorResend { seq } => [seq, 0, 0, 0, 0, 0, 0, 0],
      Message::SensorCmd(SensorCmd { op, arg }) => {
        let [a0, a1] = arg.to_le_bytes();
        [op as u8, 0, a0, a1, 0, 0, 0, 0]
      }
      Message::SensorCmdReply(SensorCmdReply {
        op,
        status,
        value,
        extra
      }) => {
        let [v0, v1, v2, v3] = value.to_le_bytes();
        let [e0, e1] = extra.to_le_bytes();
        [op as u8, status as u8, v0, v1, v2, v3, e0, e1]
      }
    };
    data
      .get(..usize::from(class.data_len()))
      .and_then(|payload| F::new(id, payload))
      .ok_or(EncodeError::FrameRejected { class })
  }

  /// The message a frame carries.
  ///
  /// # Errors
  ///
  /// A [`DecodeError`] naming the first thing wrong: an extended identifier or a remote frame,
  /// an undefined class, a payload of the wrong length, an axis byte that disagrees with the identifier,
  /// or a byte outside the values its field defines.
  pub fn decode<F: Frame>(frame: &F) -> Result<Self, DecodeError> {
    let id = match frame.id() {
      Id::Standard(id) => id,
      Id::Extended(id) => return Err(DecodeError::ExtendedId { raw: id.as_raw() })
    };
    if frame.is_remote_frame() {
      return Err(DecodeError::RemoteFrame { raw: id.as_raw() });
    }
    let fields = split_id(id);
    let class = Class::from_bits(fields.class_bits).ok_or(DecodeError::UnknownClass {
      class_bits: fields.class_bits
    })?;
    let data = frame.data();
    let wrong_length = DecodeError::WrongLength {
      class,
      expected: class.data_len(),
      got: data.len()
    };
    // A heartbeat's length is judged by `decode_heartbeat`: a version 1 heartbeat is a byte shorter, and is read
    // as version 1 so that the version check reports the mismatch instead of the frame being dropped as malformed.
    if class != Class::Heartbeat && data.len() != usize::from(class.data_len()) {
      return Err(wrong_length);
    }
    // Checks the payload's axis byte against the identifier and returns the channel.
    let channel = |payload: u8| {
      if payload == fields.channel.get() {
        Ok(fields.channel)
      } else {
        Err(DecodeError::ChannelMismatch {
          class,
          id: fields.channel.get(),
          payload
        })
      }
    };

    let message = match (class, data) {
      (Class::Estop, &[reason]) => Message::Estop { reason },
      (Class::Stop, &[reason]) => Message::Stop { reason },
      (Class::Inhibit, &[inhibited, source]) => {
        let inhibited = match inhibited {
          0 => false,
          1 => true,
          value => return Err(DecodeError::NotABool { value })
        };
        Message::Inhibit { inhibited, source }
      }
      (Class::Fault, &[axis, code, d0, d1]) => Message::Fault {
        channel: channel(axis)?,
        code: FaultCode(code),
        detail: u16::from_le_bytes([d0, d1])
      },
      (Class::MoveRel, &[seq, axis, d0, d1, d2, d3, flags]) => {
        if flags & !MOVE_FLAG_FINISH_PLUS != 0 {
          return Err(DecodeError::UndefinedMoveFlags { flags });
        }
        Message::MoveRel(MoveRel {
          seq,
          channel: channel(axis)?,
          delta_steps: i32::from_le_bytes([d0, d1, d2, d3]),
          finish_plus: flags & MOVE_FLAG_FINISH_PLUS != 0
        })
      }
      (Class::MoveDone, &[seq, axis, result, _pad, p0, p1, p2, p3]) => Message::MoveDone(MoveDone {
        seq,
        channel: channel(axis)?,
        result: MoveResult::from_byte(result).ok_or(DecodeError::UnknownMoveResult { value: result })?,
        pos_steps: i32::from_le_bytes([p0, p1, p2, p3])
      }),
      (Class::Status, &[axis, flags, p0, p1, p2, p3, c0, c1]) => Message::Status(Status {
        channel: channel(axis)?,
        flags: StatusFlags::from_bits_retain(flags),
        pos_steps: i32::from_le_bytes([p0, p1, p2, p3]),
        current_ma: u16::from_le_bytes([c0, c1])
      }),
      (Class::Heartbeat, _) => decode_heartbeat(data).ok_or(wrong_length)?,
      (Class::Config, &[key, axis, _pad0, _pad1, v0, v1, v2, v3]) => Message::Config {
        key: ConfigKey::from_byte(key).ok_or(DecodeError::UnknownConfigKey { key })?,
        channel: channel(axis)?,
        value: i32::from_le_bytes([v0, v1, v2, v3])
      },
      (Class::Sensor | Class::Shot | Class::SensorResend | Class::SensorCmd | Class::SensorCmdReply, _) => {
        decode_sensor(class, data)?.ok_or(wrong_length)?
      }
      // The length was checked against the class above, so every class matched one arm.
      _ => return Err(wrong_length)
    };

    if message.channel().is_none() && fields.channel != Channel::ZERO {
      return Err(DecodeError::UnexpectedChannel {
        class,
        id: fields.channel.get()
      });
    }
    Ok(Self {
      node: fields.node,
      message
    })
  }
}

/// A heartbeat payload: seven bytes, or six from a version 1 sender, which is then read as version 1.
fn decode_heartbeat(data: &[u8]) -> Option<Message> {
  let (uptime, state, fw_version, proto_version) = match *data {
    [u0, u1, u2, u3, state, fw_version] => ([u0, u1, u2, u3], state, fw_version, 1),
    [u0, u1, u2, u3, state, fw_version, version] => ([u0, u1, u2, u3], state, fw_version, version),
    _ => return None
  };
  Some(Message::Heartbeat {
    uptime_ms: u32::from_le_bytes(uptime),
    state,
    fw_version,
    proto_version
  })
}

/// The sensor classes' payloads (`docs/SENSOR_PLAN.md` section 3). `Ok(None)` for a class or length that is not one
/// of them, which the caller has already ruled out.
fn decode_sensor(class: Class, data: &[u8]) -> Result<Option<Message>, DecodeError> {
  Ok(Some(match (class, data) {
    (Class::Sensor, &[seq, status, h0, h1, v0, v1, s0, s1]) => Message::Sensor(SensorReading {
      seq,
      status: SensorStatus::from_byte(status).ok_or(DecodeError::UnknownSensorResult { code: status & 0x1F })?,
      h_q15: i16::from_le_bytes([h0, h1]),
      v_q15: i16::from_le_bytes([v0, v1]),
      sum_q: u16::from_le_bytes([s0, s1])
    }),
    (Class::Shot, &[seq, point, a0, a1]) => {
      let age = u16::from_le_bytes([a0, a1]);
      Message::Shot(Shot {
        seq,
        // The point is only logged: an unknown one is carried as Other rather than losing the fire.
        point: ShotPoint::from_byte(point).unwrap_or(ShotPoint::Other),
        edge_age_10us: (age != EDGE_AGE_UNKNOWN).then_some(age)
      })
    }
    (Class::SensorResend, &[seq]) => Message::SensorResend { seq },
    (Class::SensorCmd, &[op, _pad, a0, a1]) => Message::SensorCmd(SensorCmd {
      op: SensorOp::from_byte(op).ok_or(DecodeError::UnknownSensorOp { op })?,
      arg: u16::from_le_bytes([a0, a1])
    }),
    (Class::SensorCmdReply, &[op, status, v0, v1, v2, v3, e0, e1]) => Message::SensorCmdReply(SensorCmdReply {
      op: SensorOp::from_byte(op).ok_or(DecodeError::UnknownSensorOp { op })?,
      status: CmdStatus::from_byte(status).ok_or(DecodeError::UnknownCmdStatus { value: status })?,
      value: i32::from_le_bytes([v0, v1, v2, v3]),
      extra: u16::from_le_bytes([e0, e1])
    }),
    _ => return Ok(None)
  }))
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;
  use crate::sensor::SensorResult;
  use crate::test_frame::TestFrame;

  fn ch(value: u8) -> Channel {
    Channel::new(value).unwrap()
  }

  fn frame(id: u16, payload: &[u8]) -> TestFrame {
    TestFrame::standard(id, payload)
  }

  fn encode(envelope: Envelope) -> TestFrame {
    envelope.encode::<TestFrame>().unwrap()
  }

  fn decode(id: u16, payload: &[u8]) -> Result<Envelope, DecodeError> {
    Envelope::decode(&frame(id, payload))
  }

  #[test]
  fn move_rel_matches_the_byte_layout_of_the_reference_5_4_example() {
    // encode_move_rel(seq, axis, delta, plus) in reference 5.4: [seq, axis, d0, d1, d2, d3, plus as u8].
    let delta_steps = -1234_i32;
    let move_rel = MoveRel {
      seq: 9,
      channel: ch(1),
      delta_steps,
      finish_plus: true
    };
    let encoded = encode(Envelope {
      node: NodeId::MIRROR_1,
      message: Message::MoveRel(move_rel)
    });
    let d = delta_steps.to_le_bytes();
    assert_eq!(encoded.data(), &[9, 1, d[0], d[1], d[2], d[3], 1]);
    assert_eq!(encoded.raw_id(), (0x3 << 7) | (1 << 3) | 1);
  }

  #[test]
  fn multi_byte_fields_are_little_endian() {
    let status = Status {
      channel: ch(0),
      flags: StatusFlags::empty(),
      pos_steps: 0x0403_0201,
      current_ma: 0x0605
    };
    let encoded = encode(Envelope {
      node: NodeId::MIRROR_2,
      message: Message::Status(status)
    });
    assert_eq!(encoded.data(), &[0, 0, 1, 2, 3, 4, 5, 6]);
  }

  #[test]
  fn every_encoded_payload_has_the_length_its_class_names() {
    for message in examples() {
      let encoded = encode(Envelope {
        node: NodeId::MIRROR_1,
        message
      });
      assert_eq!(
        encoded.data().len(),
        usize::from(message.class().data_len()),
        "{message:?}"
      );
    }
  }

  #[test]
  fn every_example_message_survives_a_round_trip() {
    for message in examples() {
      let envelope = Envelope {
        node: NodeId::SENSOR,
        message
      };
      assert_eq!(Envelope::decode(&encode(envelope)), Ok(envelope));
    }
  }

  #[test]
  fn the_status_flags_have_the_bit_meanings_of_addendum_a_8() {
    assert_eq!(StatusFlags::MOVING.bits(), 0b0000_0001);
    assert_eq!(StatusFlags::AT_TARGET.bits(), 0b0000_0010);
    assert_eq!(StatusFlags::SUSPECT.bits(), 0b0000_0100);
    assert_eq!(StatusFlags::RAIL_OK.bits(), 0b0000_1000);
    assert_eq!(StatusFlags::SOFT_LIMIT.bits(), 0b0001_0000);
    assert_eq!(StatusFlags::INHIBITED.bits(), 0b0010_0000);
    assert_eq!(StatusFlags::PERSISTED.bits(), 0b0100_0000);
    assert_eq!(StatusFlags::ARMED.bits(), 0b1000_0000);
    // Every bit is defined, so no received flag byte loses a bit.
    assert_eq!(StatusFlags::all().bits(), 0xFF);
  }

  #[test]
  fn the_move_results_and_config_keys_have_the_numbers_of_addendum_a_8() {
    assert_eq!(
      [
        MoveResult::Done,
        MoveResult::Stopped,
        MoveResult::RailLost,
        MoveResult::SoftLimit
      ]
      .map(|r| r as u8),
      [0, 1, 2, 3]
    );
    let keys = [
      ConfigKey::StepPeriodUs,
      ConfigKey::OvershootSteps,
      ConfigKey::SoftMinSteps,
      ConfigKey::SoftMaxSteps,
      ConfigKey::DirSign
    ];
    assert_eq!(keys.map(|k| k as u8), [1, 2, 3, 4, 5]);
    assert_eq!(FaultCode::AXIS_NOT_RESPONDING.0, 0x10);
    assert_eq!(FaultCode::RAIL_LOST_DURING_MOVE.0, 0x11);
  }

  #[test]
  fn a_frame_with_an_extended_identifier_is_rejected() {
    let id = embedded_can::ExtendedId::new(0x180).unwrap();
    let extended = TestFrame::new(id, &[0; 7]).unwrap();
    assert_eq!(Envelope::decode(&extended), Err(DecodeError::ExtendedId { raw: 0x180 }));
  }

  #[test]
  fn a_remote_frame_is_rejected() {
    let id = embedded_can::StandardId::new(0x180).unwrap();
    let remote = TestFrame::new_remote(id, 7).unwrap();
    assert_eq!(Envelope::decode(&remote), Err(DecodeError::RemoteFrame { raw: 0x180 }));
  }

  #[test]
  fn a_frame_type_that_refuses_the_frame_is_an_encode_error() {
    /// A frame type that refuses everything, as one with tighter limits than CAN 2.0 might.
    struct Refusing;
    impl Frame for Refusing {
      fn new(_: impl Into<Id>, _: &[u8]) -> Option<Self> {
        None
      }
      fn new_remote(_: impl Into<Id>, _: usize) -> Option<Self> {
        None
      }
      fn is_extended(&self) -> bool {
        false
      }
      fn is_remote_frame(&self) -> bool {
        false
      }
      fn id(&self) -> Id {
        Id::Standard(embedded_can::StandardId::ZERO)
      }
      fn dlc(&self) -> usize {
        0
      }
      fn data(&self) -> &[u8] {
        &[]
      }
    }
    let envelope = Envelope {
      node: NodeId::MIRROR_1,
      message: Message::Stop { reason: 0 }
    };
    assert!(matches!(
      envelope.encode::<Refusing>(),
      Err(EncodeError::FrameRejected { class: Class::Stop })
    ));
  }

  #[test]
  fn shot_matches_the_layout_of_the_sensor_plan() {
    let shot = Shot {
      seq: 0x21,
      point: ShotPoint::B,
      edge_age_10us: Some(0x0302)
    };
    let encoded = encode(Envelope {
      node: NodeId::COORDINATOR,
      message: Message::Shot(shot)
    });
    assert_eq!(encoded.raw_id(), 0xA << 7);
    assert_eq!(encoded.data(), &[0x21, 1, 0x02, 0x03]);
  }

  #[test]
  fn an_unknown_edge_age_travels_as_0xffff_and_a_known_one_saturates_below_it() {
    let unknown = Shot {
      seq: 1,
      point: ShotPoint::Other,
      edge_age_10us: None
    };
    let encoded = encode(Envelope {
      node: NodeId::COORDINATOR,
      message: Message::Shot(unknown)
    });
    assert_eq!(encoded.data(), &[1, 0xFF, 0xFF, 0xFF]);
    assert_eq!(Envelope::decode(&encoded).unwrap().message, Message::Shot(unknown));

    let at_limit = Shot {
      edge_age_10us: Some(0xFFFF),
      ..unknown
    };
    let decoded = Envelope::decode(&encode(Envelope {
      node: NodeId::COORDINATOR,
      message: Message::Shot(at_limit)
    }));
    assert_eq!(
      decoded.unwrap().message,
      Message::Shot(Shot {
        edge_age_10us: Some(0xFFFE),
        ..unknown
      })
    );
  }

  #[test]
  fn sensor_matches_the_layout_of_the_sensor_plan() {
    let reading = SensorReading {
      seq: 5,
      status: SensorStatus {
        result: SensorResult::Saturated,
        calibrated: true,
        bench: false
      },
      h_q15: 0x0201,
      v_q15: -2,
      sum_q: 0x8000
    };
    let encoded = encode(Envelope {
      node: NodeId::SENSOR,
      message: Message::Sensor(reading)
    });
    assert_eq!(encoded.raw_id(), (0x7 << 7) | (3 << 3));
    assert_eq!(encoded.data(), &[5, 0b0010_0010, 0x01, 0x02, 0xFE, 0xFF, 0x00, 0x80]);
  }

  #[test]
  fn a_sensor_reading_with_an_undefined_result_is_rejected_and_the_reserved_bit_is_not() {
    let id = (0x7 << 7) | (3 << 3);
    assert_eq!(
      decode(id, &[0, 5, 0, 0, 0, 0, 0, 0]),
      Err(DecodeError::UnknownSensorResult { code: 5 })
    );
    assert_eq!(
      decode(id, &[0, 0b1010_1000, 0, 0, 0, 0, 0, 0]),
      Err(DecodeError::UnknownSensorResult { code: 8 })
    );
    let with_reserved = decode(id, &[0, 0b0100_0000, 0, 0, 0, 0, 0, 0]).unwrap();
    let expected = SensorStatus {
      result: SensorResult::Valid,
      calibrated: false,
      bench: false
    };
    assert!(matches!(with_reserved.message, Message::Sensor(SensorReading { status, .. }) if status == expected));
  }

  #[test]
  fn undefined_sensor_ops_and_command_statuses_are_rejected_and_an_unknown_point_is_carried() {
    let unknown_point = decode(0xA << 7, &[4, 3, 0, 0]).unwrap();
    let expected = Shot {
      seq: 4,
      point: ShotPoint::Other,
      edge_age_10us: Some(0)
    };
    assert_eq!(
      unknown_point.message,
      Message::Shot(expected),
      "an unknown point is carried as Other"
    );
    let sensor = 3 << 3;
    assert_eq!(
      decode((0xC << 7) | sensor, &[0, 0, 0, 0]),
      Err(DecodeError::UnknownSensorOp { op: 0 })
    );
    assert_eq!(
      decode((0xC << 7) | sensor, &[7, 0, 0, 0]),
      Err(DecodeError::UnknownSensorOp { op: 7 })
    );
    assert_eq!(
      decode((0xD << 7) | sensor, &[9, 0, 0, 0, 0, 0, 0, 0]),
      Err(DecodeError::UnknownSensorOp { op: 9 })
    );
    assert_eq!(
      decode((0xD << 7) | sensor, &[1, 4, 0, 0, 0, 0, 0, 0]),
      Err(DecodeError::UnknownCmdStatus { value: 4 })
    );
  }

  #[test]
  fn the_heartbeat_carries_the_protocol_version_in_its_seventh_byte() {
    let heartbeat = Message::Heartbeat {
      uptime_ms: 0x0403_0201,
      state: 1,
      fw_version: 9,
      proto_version: 2
    };
    let encoded = encode(Envelope {
      node: NodeId::MIRROR_1,
      message: heartbeat
    });
    assert_eq!(encoded.data(), &[1, 2, 3, 4, 1, 9, 2]);
    // A version 1 heartbeat, six bytes long, reads as version 1, so that the version check can report it.
    let legacy = decode(0x8 << 7, &[1, 0, 0, 0, 2, 3]).unwrap();
    let expected = Message::Heartbeat {
      uptime_ms: 1,
      state: 2,
      fw_version: 3,
      proto_version: 1
    };
    assert_eq!(legacy.message, expected);
    let five = DecodeError::WrongLength {
      class: Class::Heartbeat,
      expected: 7,
      got: 5
    };
    assert_eq!(decode(0x8 << 7, &[0; 5]), Err(five));
  }

  #[test]
  fn the_sensor_fault_codes_follow_the_motor_node_s() {
    let codes = [
      FaultCode::FRONT_END_FAULT,
      FaultCode::BUFFER_OVERRUN,
      FaultCode::CALIBRATION_MISSING,
      FaultCode::UNEXPECTED_FIRE,
      FaultCode::PROTOCOL_MISMATCH
    ];
    assert_eq!(codes.map(|code| code.0), [0x30, 0x31, 0x32, 0x33, 0x34]);
  }

  #[test]
  fn an_undefined_class_is_rejected() {
    assert_eq!(
      decode(0xE << 7, &[]),
      Err(DecodeError::UnknownClass { class_bits: 0xE })
    );
    assert_eq!(
      decode(0x7FF, &[0; 8]),
      Err(DecodeError::UnknownClass { class_bits: 0xF })
    );
  }

  #[test]
  fn a_payload_one_byte_short_or_long_is_rejected() {
    let id = 0x3 << 7;
    let short = DecodeError::WrongLength {
      class: Class::MoveRel,
      expected: 7,
      got: 6
    };
    assert_eq!(decode(id, &[0; 6]), Err(short));
    let long = DecodeError::WrongLength {
      class: Class::MoveRel,
      expected: 7,
      got: 8
    };
    assert_eq!(decode(id, &[0; 8]), Err(long));
  }

  #[test]
  fn an_axis_byte_that_disagrees_with_the_identifier_is_rejected() {
    let id = (0x3 << 7) | (1 << 3) | 1;
    assert_eq!(
      decode(id, &[0, 2, 0, 0, 0, 0, 0]),
      Err(DecodeError::ChannelMismatch {
        class: Class::MoveRel,
        id: 1,
        payload: 2
      })
    );
    // A payload axis above the 3-bit field can never match it.
    assert_eq!(
      decode(id, &[0, 9, 0, 0, 0, 0, 0]),
      Err(DecodeError::ChannelMismatch {
        class: Class::MoveRel,
        id: 1,
        payload: 9
      })
    );
  }

  #[test]
  fn a_message_without_a_channel_rejects_a_nonzero_axis_field() {
    assert_eq!(
      decode(3, &[0]),
      Err(DecodeError::UnexpectedChannel {
        class: Class::Estop,
        id: 3
      })
    );
  }

  #[test]
  fn an_inhibit_byte_other_than_zero_or_one_is_rejected() {
    assert_eq!(decode(0x1 << 7, &[2, 0]), Err(DecodeError::NotABool { value: 2 }));
  }

  #[test]
  fn a_move_with_undefined_flag_bits_is_rejected() {
    let id = 0x3 << 7;
    assert_eq!(
      decode(id, &[0, 0, 5, 0, 0, 0, 0b11]),
      Err(DecodeError::UndefinedMoveFlags { flags: 0b11 })
    );
    assert_eq!(
      decode(id, &[0, 0, 5, 0, 0, 0, 0x80]),
      Err(DecodeError::UndefinedMoveFlags { flags: 0x80 })
    );
  }

  #[test]
  fn an_undefined_move_result_is_rejected() {
    assert_eq!(
      decode(0x5 << 7, &[0, 0, 4, 0, 0, 0, 0, 0]),
      Err(DecodeError::UnknownMoveResult { value: 4 })
    );
  }

  #[test]
  fn an_undefined_config_key_is_rejected() {
    assert_eq!(
      decode(0x9 << 7, &[0, 0, 0, 0, 0, 0, 0, 0]),
      Err(DecodeError::UnknownConfigKey { key: 0 })
    );
    assert_eq!(
      decode(0x9 << 7, &[6, 0, 0, 0, 0, 0, 0, 0]),
      Err(DecodeError::UnknownConfigKey { key: 6 })
    );
  }

  #[test]
  fn reserved_bytes_are_written_as_zero_and_ignored_when_read() {
    let done = MoveDone {
      seq: 1,
      channel: ch(0),
      result: MoveResult::Done,
      pos_steps: 7
    };
    let encoded = encode(Envelope {
      node: NodeId::MIRROR_1,
      message: Message::MoveDone(done)
    });
    assert_eq!(encoded.data().get(3), Some(&0));
    let mut with_pad = [0_u8; 8];
    with_pad.copy_from_slice(encoded.data());
    with_pad[3] = 0xAA;
    let decoded = Envelope::decode(&frame(encoded.raw_id(), &with_pad)).unwrap();
    assert_eq!(decoded.message, Message::MoveDone(done));
  }

  #[test]
  fn an_unknown_fault_code_is_carried_through() {
    let decoded = decode((0x2 << 7) | (1 << 3), &[0, 0x7E, 0x34, 0x12]).unwrap();
    assert_eq!(
      decoded.message,
      Message::Fault {
        channel: ch(0),
        code: FaultCode(0x7E),
        detail: 0x1234
      }
    );
  }

  fn examples() -> [Message; 14] {
    [
      Message::Estop { reason: 3 },
      Message::Inhibit {
        inhibited: true,
        source: 2
      },
      Message::Fault {
        channel: ch(1),
        code: FaultCode::RAIL_LOST_DURING_MOVE,
        detail: 0xBEEF
      },
      Message::MoveRel(MoveRel {
        seq: 255,
        channel: ch(2),
        delta_steps: i32::MIN,
        finish_plus: false
      }),
      Message::Stop { reason: 1 },
      Message::MoveDone(MoveDone {
        seq: 0,
        channel: ch(1),
        result: MoveResult::RailLost,
        pos_steps: -5
      }),
      Message::Status(Status {
        channel: ch(0),
        flags: StatusFlags::SUSPECT | StatusFlags::RAIL_OK,
        pos_steps: i32::MAX,
        current_ma: 0
      }),
      Message::Sensor(SensorReading {
        seq: 7,
        status: SensorStatus {
          result: SensorResult::NoShot,
          calibrated: true,
          bench: false
        },
        h_q15: i16::MIN,
        v_q15: i16::MAX,
        sum_q: u16::MAX
      }),
      Message::Heartbeat {
        uptime_ms: u32::MAX,
        state: 4,
        fw_version: 1,
        proto_version: crate::PROTOCOL_VERSION
      },
      Message::Config {
        key: ConfigKey::DirSign,
        channel: ch(1),
        value: -1
      },
      Message::Shot(Shot {
        seq: 200,
        point: ShotPoint::C,
        edge_age_10us: Some(1_234)
      }),
      Message::SensorResend { seq: 9 },
      Message::SensorCmd(SensorCmd {
        op: SensorOp::Report,
        arg: 3
      }),
      Message::SensorCmdReply(SensorCmdReply {
        op: SensorOp::Report,
        status: CmdStatus::Ok,
        value: 98_500,
        extra: 41
      })
    ]
  }

  fn any_channel() -> impl Strategy<Value = Channel> {
    (0_u8..=Channel::MAX).prop_map(ch)
  }

  fn any_message() -> impl Strategy<Value = Message> {
    let result = prop_oneof![
      Just(MoveResult::Done),
      Just(MoveResult::Stopped),
      Just(MoveResult::RailLost),
      Just(MoveResult::SoftLimit)
    ];
    let key = prop_oneof![
      Just(ConfigKey::StepPeriodUs),
      Just(ConfigKey::OvershootSteps),
      Just(ConfigKey::SoftMinSteps),
      Just(ConfigKey::SoftMaxSteps),
      Just(ConfigKey::DirSign)
    ];
    prop_oneof![
      any::<u8>().prop_map(|reason| Message::Estop { reason }),
      (any::<bool>(), any::<u8>()).prop_map(|(inhibited, source)| Message::Inhibit { inhibited, source }),
      (any_channel(), any::<u8>(), any::<u16>()).prop_map(|(channel, code, detail)| Message::Fault {
        channel,
        code: FaultCode(code),
        detail
      }),
      (any::<u8>(), any_channel(), any::<i32>(), any::<bool>()).prop_map(|(seq, channel, delta_steps, finish_plus)| {
        Message::MoveRel(MoveRel {
          seq,
          channel,
          delta_steps,
          finish_plus
        })
      }),
      any::<u8>().prop_map(|reason| Message::Stop { reason }),
      (any::<u8>(), any_channel(), result, any::<i32>()).prop_map(|(seq, channel, result, pos_steps)| {
        Message::MoveDone(MoveDone {
          seq,
          channel,
          result,
          pos_steps
        })
      }),
      (any_channel(), any::<u8>(), any::<i32>(), any::<u16>()).prop_map(|(channel, flags, pos_steps, current_ma)| {
        Message::Status(Status {
          channel,
          flags: StatusFlags::from_bits_retain(flags),
          pos_steps,
          current_ma
        })
      }),
      (any::<u32>(), any::<u8>(), any::<u8>(), any::<u8>()).prop_map(
        |(uptime_ms, state, fw_version, proto_version)| Message::Heartbeat {
          uptime_ms,
          state,
          fw_version,
          proto_version
        }
      ),
      (key, any_channel(), any::<i32>()).prop_map(|(key, channel, value)| Message::Config { key, channel, value }),
      any_sensor_message(),
    ]
  }

  fn any_sensor_status() -> impl Strategy<Value = SensorStatus> {
    let result = proptest::sample::select(
      &[
        SensorResult::Valid,
        SensorResult::NoShot,
        SensorResult::Saturated,
        SensorResult::NegativeQuadrant,
        SensorResult::BaselineUnstable,
        SensorResult::Overlap,
        SensorResult::TimedOut,
        SensorResult::FrontEndFault
      ][..]
    );
    (result, any::<bool>(), any::<bool>()).prop_map(|(result, calibrated, bench)| SensorStatus {
      result,
      calibrated,
      bench
    })
  }

  fn any_sensor_op() -> impl Strategy<Value = SensorOp> {
    proptest::sample::select(
      &[
        SensorOp::Noise,
        SensorOp::Tau,
        SensorOp::SRef,
        SensorOp::Save,
        SensorOp::Report,
        SensorOp::Bench
      ][..]
    )
  }

  /// The sensor's messages. A known edge age stays below the "unknown" wire value, which it can never round-trip as.
  fn any_sensor_message() -> impl Strategy<Value = Message> {
    let point = proptest::sample::select(&[ShotPoint::A, ShotPoint::B, ShotPoint::C, ShotPoint::Other][..]);
    let age = proptest::option::of(0_u16..EDGE_AGE_UNKNOWN);
    let status = proptest::sample::select(&[CmdStatus::Ok, CmdStatus::Busy, CmdStatus::Refused, CmdStatus::Failed][..]);
    prop_oneof![
      (
        any::<u8>(),
        any_sensor_status(),
        any::<i16>(),
        any::<i16>(),
        any::<u16>()
      )
        .prop_map(|(seq, status, h_q15, v_q15, sum_q)| {
          Message::Sensor(SensorReading {
            seq,
            status,
            h_q15,
            v_q15,
            sum_q
          })
        }),
      (any::<u8>(), point, age).prop_map(|(seq, point, edge_age_10us)| Message::Shot(Shot {
        seq,
        point,
        edge_age_10us
      })),
      any::<u8>().prop_map(|seq| Message::SensorResend { seq }),
      (any_sensor_op(), any::<u16>()).prop_map(|(op, arg)| Message::SensorCmd(SensorCmd { op, arg })),
      (any_sensor_op(), status, any::<i32>(), any::<u16>()).prop_map(|(op, status, value, extra)| {
        Message::SensorCmdReply(SensorCmdReply {
          op,
          status,
          value,
          extra
        })
      }),
    ]
  }

  proptest! {
    #[test]
    fn every_message_survives_a_round_trip(node in 0_u8..=NodeId::MAX, message in any_message()) {
      let envelope = Envelope { node: NodeId::new(node).unwrap(), message };
      prop_assert_eq!(Envelope::decode(&encode(envelope)), Ok(envelope));
    }

    #[test]
    fn a_frame_that_decodes_encodes_back_to_itself_up_to_reserved_bytes(
      id in 0_u16..=0x7FF, payload in proptest::collection::vec(any::<u8>(), 0..=8),
    ) {
      // Decoding never panics, whatever the bus delivers; and what it accepts it reproduces.
      let received = frame(id, &payload);
      if let Ok(envelope) = Envelope::decode(&received) {
        let sent = encode(envelope);
        prop_assert_eq!(sent.raw_id(), received.raw_id());
        // A version 1 heartbeat is re-encoded in the current, one byte longer layout.
        let legacy = matches!(envelope.message, Message::Heartbeat { proto_version: 1, .. }) && received.data().len() == 6;
        if !legacy {
          prop_assert_eq!(sent.data().len(), received.data().len());
        }
        prop_assert_eq!(Envelope::decode(&sent), Ok(envelope));
      }
    }
  }
}
