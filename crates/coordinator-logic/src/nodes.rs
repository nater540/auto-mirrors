//! The coordinator's watch over the nodes on the bus, from their heartbeats (reference 5.4).
//!
//! Every node sends a heartbeat at the coordinator's period. The coordinator judges them the way each node judges
//! the coordinator, so that both sides agree on what a loss and a restart are:
//!
//! - A node is **lost** when its last heartbeat is older than `heartbeat_timeout_ms`. The state machine is told
//!   (`align_core::coordinator::Event::NodeLost`), which ends a move.
//! - A node **restarted** when its uptime disagrees with the local clock, since its last heartbeat, by more than
//!   the same timeout. Comparing with wrapping arithmetic keeps the 49.7-day wrap of the `u32` uptime from looking
//!   like one. A motor node then needs every channel's CONFIG again, because it keeps soft limits and direction
//!   signs in RAM, and the moves it was running are gone with its duplicate-move tracker.
//! - A heartbeat of another protocol version does not count: the node is not present, and the mismatch is reported
//!   once for each version seen, as the nodes report one from the coordinator (FAULT 0x34).
//!
//! Every change is reported once, when it happens; [`Nodes::present`] gives the current answer at any time.

use align_core::config::NodeTiming;
use align_proto::{NodeId, PROTOCOL_VERSION};

/// The nodes the coordinator expects on the bus: the two motor nodes and the sensor node.
pub const WATCHED: [NodeId; 3] = [NodeId::MIRROR_1, NodeId::MIRROR_2, NodeId::SENSOR];

/// What changed about a node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
  /// The node is present: its first heartbeat, or the first after it was lost, without a sign of a restart.
  Appeared,
  /// The node's uptime says it restarted since its last heartbeat.
  /// It is present, and it forgot its configuration and its moves.
  Restarted,
  /// No heartbeat within the timeout. The node is no longer present.
  Lost,
  /// A heartbeat of another protocol version. The node is not present; the caller treats it as lost.
  ProtocolMismatch {
    /// The version the node sent.
    version: u8
  }
}

/// What is known about one node.
#[derive(Clone, Copy, Debug, Default)]
struct Watch {
  /// The local time of the last heartbeat that counted.
  last_seen_ms: Option<u64>,
  /// The uptime that heartbeat carried.
  uptime_ms: Option<u32>,
  present: bool,
  /// The protocol version last reported as a mismatch, so that it is reported once.
  mismatched: Option<u8>
}

/// The watch over every node in [`WATCHED`].
#[derive(Clone, Debug)]
pub struct Nodes {
  timeout_ms: u32,
  watches: [Watch; WATCHED.len()]
}

impl Nodes {
  /// No node seen yet.
  pub fn new(timing: &NodeTiming) -> Self {
    Self {
      timeout_ms: timing.heartbeat_timeout_ms,
      watches: [Watch::default(); WATCHED.len()]
    }
  }

  /// A heartbeat from `node`, at the local time `now_ms`. Returns what changed, if anything.
  /// A node that is not watched changes nothing.
  pub fn on_heartbeat(&mut self, now_ms: u64, node: NodeId, uptime_ms: u32, proto_version: u8) -> Option<Change> {
    let timeout_ms = self.timeout_ms;
    let watch = self.watch_mut(node)?;
    if proto_version != PROTOCOL_VERSION {
      let first = watch.mismatched != Some(proto_version);
      // Forgotten, so that the next heartbeat in this version is judged as a node that appears.
      *watch = Watch {
        mismatched: Some(proto_version),
        ..Watch::default()
      };
      return first.then_some(Change::ProtocolMismatch { version: proto_version });
    }
    watch.mismatched = None;
    let restarted = match (watch.last_seen_ms, watch.uptime_ms) {
      (Some(seen_ms), Some(previous_ms)) => {
        let remote_ms = i64::from(uptime_ms.wrapping_sub(previous_ms));
        let local_ms = i64::try_from(now_ms.saturating_sub(seen_ms)).unwrap_or(i64::MAX);
        remote_ms.abs_diff(local_ms) > u64::from(timeout_ms)
      }
      _ => false
    };
    let change = if restarted {
      Some(Change::Restarted)
    } else if watch.present {
      None
    } else {
      Some(Change::Appeared)
    };
    watch.last_seen_ms = Some(now_ms);
    watch.uptime_ms = Some(uptime_ms);
    watch.present = true;
    change
  }

  /// Checks every node for a heartbeat older than the timeout, at the local time `now_ms`. Returns, for each node
  /// in [`WATCHED`] order, [`Change::Lost`] for one that was present until now.
  pub fn tick(&mut self, now_ms: u64) -> [Option<Change>; WATCHED.len()] {
    let timeout_ms = u64::from(self.timeout_ms);
    let mut changes = [None; WATCHED.len()];
    for (watch, change) in self.watches.iter_mut().zip(changes.iter_mut()) {
      let stale = watch
        .last_seen_ms
        .is_some_and(|seen_ms| now_ms.saturating_sub(seen_ms) > timeout_ms);
      if watch.present && stale {
        watch.present = false;
        *change = Some(Change::Lost);
      }
    }
    changes
  }

  /// Whether `node` is present:
  /// a heartbeat of this protocol version within the timeout, as of the last [`Nodes::tick`].
  pub fn present(&self, node: NodeId) -> bool {
    self.watch(node).is_some_and(|watch| watch.present)
  }

  /// Whether every watched node is present.
  pub fn all_present(&self) -> bool {
    self.watches.iter().all(|watch| watch.present)
  }

  fn index(node: NodeId) -> Option<usize> {
    WATCHED.iter().position(|watched| *watched == node)
  }

  fn watch(&self, node: NodeId) -> Option<&Watch> {
    self.watches.get(Self::index(node)?)
  }

  fn watch_mut(&mut self, node: NodeId) -> Option<&mut Watch> {
    self.watches.get_mut(Self::index(node)?)
  }
}

#[cfg(test)]
mod tests;
