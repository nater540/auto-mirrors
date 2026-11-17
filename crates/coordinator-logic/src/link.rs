//! The coordinator's own traffic, and what it has heard from each node (reference 5.4).
//!
//! - **The heartbeat** goes out every `heartbeat_period_ms`, the first at once. A node stops and disarms when the
//!   last one is older than `heartbeat_timeout_ms`, so a coordinator that stops sending stops every motor.
//! - **The inhibit level** goes out with every heartbeat, and at once when it changes. The repeat is what tells a
//!   node that restarted. A node boots inhibited, so until the first INHIBIT 0 the level only confirms what every
//!   node already assumes.
//! - **What was heard:** the watch over the nodes ([`crate::nodes`]) judges each heartbeat. This module keeps the
//!   rest for the console: each node's last heartbeat (state, firmware version, uptime) and the last STATUS of each
//!   of its channels.
//!
//! Every frame sent here is in the coordinator's name ([`NodeId::COORDINATOR`]), and no frame in that name is taken
//! for a node's.

use align_core::config::NodeTiming;
use align_proto::{Channel, Envelope, Message, NodeId, PROTOCOL_VERSION, Status};

use crate::nodes::{Change, Nodes, WATCHED};

/// Channels a node can report, by number (0 to [`Channel::MAX`]).
pub const CHANNELS: usize = Channel::MAX as usize + 1;

/// What the coordinator says about itself on the bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Beacon {
  /// The state byte of its heartbeat.
  pub state: u8,
  /// Whether motion is inhibited: the negation of `align_core::coordinator::Coordinator::motion_allowed`.
  pub inhibited: bool
}

/// What was last heard from one node. Kept after the node is lost, as what it said last.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Heard {
  /// The state byte of its last heartbeat, whose values each node's firmware defines.
  pub state: Option<u8>,
  /// The firmware version of its last heartbeat.
  pub fw_version: Option<u8>,
  /// The uptime of its last heartbeat, in milliseconds.
  pub uptime_ms: Option<u32>,
  /// The last STATUS of each channel, by channel number.
  pub status: [Option<Status>; CHANNELS]
}

/// The coordinator's side of the bus outside of moves and readings.
#[derive(Clone, Debug)]
pub struct Link {
  nodes: Nodes,
  fw_version: u8,
  period_ms: u64,
  next_heartbeat_ms: u64,
  /// The inhibit level last sent; `None` before the first.
  sent_inhibited: Option<bool>,
  heard: [Heard; WATCHED.len()]
}

impl Link {
  /// Nothing sent or heard yet. The first [`Link::tick`] sends the heartbeat.
  pub fn new(timing: &NodeTiming, fw_version: u8) -> Self {
    Self {
      nodes: Nodes::new(timing),
      fw_version,
      period_ms: u64::from(timing.heartbeat_period_ms),
      next_heartbeat_ms: 0,
      sent_inhibited: None,
      heard: [Heard::default(); WATCHED.len()]
    }
  }

  /// Sends what is due at the local time `now_ms` through `send`: the heartbeat and the inhibit level once a
  /// period, and the inhibit level at once when `beacon` changes it. Returns, for each node in [`WATCHED`] order,
  /// [`Change::Lost`] for one whose heartbeat has just become older than the timeout.
  pub fn tick(
    &mut self,
    now_ms: u64,
    beacon: Beacon,
    send: &mut impl FnMut(Envelope)
  ) -> [Option<Change>; WATCHED.len()] {
    let lost = self.nodes.tick(now_ms);
    let heartbeat_due = now_ms >= self.next_heartbeat_ms;
    if heartbeat_due {
      self.next_heartbeat_ms = now_ms.saturating_add(self.period_ms);
      // The low 32 bits, wrapping as the nodes' uptime does; the nodes compare uptimes with wrapping arithmetic.
      let uptime_ms = u32::try_from(now_ms & u64::from(u32::MAX)).unwrap_or(u32::MAX);
      send(coordinator(Message::Heartbeat {
        uptime_ms,
        state: beacon.state,
        fw_version: self.fw_version,
        proto_version: PROTOCOL_VERSION
      }));
    }
    if heartbeat_due || self.sent_inhibited != Some(beacon.inhibited) {
      self.sent_inhibited = Some(beacon.inhibited);
      // The source byte has no defined values yet (CLAUDE.md, M3).
      send(coordinator(Message::Inhibit {
        inhibited: beacon.inhibited,
        source: 0
      }));
    }
    lost
  }

  /// A frame from the bus, at the local time `now_ms`. Returns what changed about its node, if anything.
  /// Only heartbeats and STATUS are kept here; the caller handles the rest.
  pub fn on_frame(&mut self, now_ms: u64, envelope: Envelope) -> Option<(NodeId, Change)> {
    let node = envelope.node;
    if node == NodeId::COORDINATOR {
      return None;
    }
    match envelope.message {
      Message::Heartbeat {
        uptime_ms,
        state,
        fw_version,
        proto_version
      } => {
        let heard = self.heard_mut(node)?;
        heard.state = Some(state);
        heard.fw_version = Some(fw_version);
        heard.uptime_ms = Some(uptime_ms);
        let change = self.nodes.on_heartbeat(now_ms, node, uptime_ms, proto_version)?;
        Some((node, change))
      }
      Message::Status(status) => {
        let slot = self
          .heard_mut(node)?
          .status
          .get_mut(usize::from(status.channel.get()))?;
        *slot = Some(status);
        None
      }
      _ => None
    }
  }

  /// Whether `node` is present, as of the last [`Link::tick`] (see [`Nodes::present`]).
  pub fn present(&self, node: NodeId) -> bool {
    self.nodes.present(node)
  }

  /// What was last heard from `node`; `None` for a node that is not in [`WATCHED`].
  pub fn heard(&self, node: NodeId) -> Option<&Heard> {
    self.heard.get(WATCHED.iter().position(|watched| *watched == node)?)
  }

  fn heard_mut(&mut self, node: NodeId) -> Option<&mut Heard> {
    self.heard.get_mut(WATCHED.iter().position(|watched| *watched == node)?)
  }
}

/// A message in the coordinator's name.
const fn coordinator(message: Message) -> Envelope {
  Envelope {
    node: NodeId::COORDINATOR,
    message
  }
}

#[cfg(test)]
mod tests;
