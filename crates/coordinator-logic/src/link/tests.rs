//! The coordinator's own traffic, one rule at a time, and what it keeps of what the nodes say.

extern crate std;

use std::vec::Vec;

use align_proto::{PROTOCOL_VERSION, StatusFlags};

use super::*;

const TIMING: NodeTiming = NodeTiming::STARTING;
const PERIOD_MS: u64 = TIMING.heartbeat_period_ms as u64;
const TIMEOUT_MS: u64 = TIMING.heartbeat_timeout_ms as u64;
const FW_VERSION: u8 = 3;
const MIRROR_1: NodeId = NodeId::MIRROR_1;

const INHIBITED: Beacon = Beacon {
  state: 0,
  inhibited: true
};

fn link() -> Link {
  Link::new(&TIMING, FW_VERSION)
}

/// The frames one tick sends.
fn tick(link: &mut Link, now_ms: u64, beacon: Beacon) -> Vec<Envelope> {
  let mut sent = Vec::new();
  link.tick(now_ms, beacon, &mut |envelope| sent.push(envelope));
  sent
}

fn heartbeat(uptime_ms: u32, state: u8) -> Envelope {
  Envelope {
    node: NodeId::COORDINATOR,
    message: Message::Heartbeat {
      uptime_ms,
      state,
      fw_version: FW_VERSION,
      proto_version: PROTOCOL_VERSION
    }
  }
}

fn inhibit(inhibited: bool) -> Envelope {
  Envelope {
    node: NodeId::COORDINATOR,
    message: Message::Inhibit { inhibited, source: 0 }
  }
}

/// A heartbeat from `node`, in this protocol version.
fn node_beat(node: NodeId, uptime_ms: u32, state: u8) -> Envelope {
  Envelope {
    node,
    message: Message::Heartbeat {
      uptime_ms,
      state,
      fw_version: 1,
      proto_version: PROTOCOL_VERSION
    }
  }
}

fn status(node: NodeId, channel: u8, pos_steps: i32) -> Envelope {
  Envelope {
    node,
    message: Message::Status(Status {
      channel: Channel::new(channel).unwrap(),
      flags: StatusFlags::empty(),
      pos_steps,
      current_ma: 0
    })
  }
}

#[test]
fn the_first_tick_sends_the_heartbeat_and_the_inhibit_level_at_once() {
  let mut link = link();
  assert_eq!(tick(&mut link, 0, INHIBITED), [heartbeat(0, 0), inhibit(true)]);
}

#[test]
fn heartbeats_go_out_once_a_period_with_the_inhibit_level_repeated() {
  let mut link = link();
  tick(&mut link, 0, INHIBITED);
  assert_eq!(tick(&mut link, PERIOD_MS - 1, INHIBITED), []);
  assert_eq!(
    tick(&mut link, PERIOD_MS, INHIBITED),
    [heartbeat(100, 0), inhibit(true)]
  );
  assert_eq!(tick(&mut link, PERIOD_MS + 50, INHIBITED), []);
  assert_eq!(
    tick(&mut link, 2 * PERIOD_MS, INHIBITED),
    [heartbeat(200, 0), inhibit(true)]
  );
}

#[test]
fn the_heartbeat_carries_the_uptime_the_state_and_both_versions() {
  let mut link = link();
  let beacon = Beacon {
    state: 4,
    inhibited: true
  };
  assert_eq!(tick(&mut link, 1_234, beacon).first(), Some(&heartbeat(1_234, 4)));
}

#[test]
fn the_uptime_wraps_like_the_nodes_and_not_saturates() {
  let mut link = link();
  let now_ms = u64::from(u32::MAX) + 6;
  assert_eq!(tick(&mut link, now_ms, INHIBITED).first(), Some(&heartbeat(5, 0)));
}

#[test]
fn a_change_of_the_inhibit_level_goes_out_at_once_between_heartbeats() {
  let mut link = link();
  tick(&mut link, 0, INHIBITED);
  let allowed = Beacon {
    state: 0,
    inhibited: false
  };
  assert_eq!(tick(&mut link, 10, allowed), [inhibit(false)]);
  assert_eq!(tick(&mut link, 20, allowed), []);
  assert_eq!(tick(&mut link, 30, INHIBITED), [inhibit(true)]);
  assert_eq!(
    tick(&mut link, PERIOD_MS, INHIBITED),
    [heartbeat(100, 0), inhibit(true)]
  );
}

#[test]
fn every_frame_it_sends_is_addressed_from_the_coordinator() {
  let mut link = link();
  let mut sent = Vec::new();
  for step in 0..20_u64 {
    let beacon = Beacon {
      state: 0,
      inhibited: step % 3 != 0
    };
    sent.extend(tick(&mut link, step * 37, beacon));
  }
  assert!(!sent.is_empty());
  assert!(sent.iter().all(|envelope| envelope.node == NodeId::COORDINATOR));
}

#[test]
fn a_node_heartbeat_is_reported_when_the_node_appears_and_kept_for_the_console() {
  let mut link = link();
  assert!(!link.present(MIRROR_1));
  assert_eq!(link.heard(MIRROR_1), Some(&Heard::default()));
  assert_eq!(
    link.on_frame(1_000, node_beat(MIRROR_1, 5_000, 2)),
    Some((MIRROR_1, Change::Appeared))
  );
  assert_eq!(link.on_frame(1_100, node_beat(MIRROR_1, 5_100, 2)), None);
  assert!(link.present(MIRROR_1));
  let heard = link.heard(MIRROR_1).unwrap();
  assert_eq!(heard.state, Some(2));
  assert_eq!(heard.fw_version, Some(1));
  assert_eq!(heard.uptime_ms, Some(5_100));
}

#[test]
fn a_silent_node_is_reported_lost_once_by_the_tick() {
  let mut link = link();
  link.on_frame(1_000, node_beat(MIRROR_1, 5_000, 2));
  // Mirror 1 comes first in WATCHED.
  let lost = |link: &mut Link, now_ms: u64| link.tick(now_ms, INHIBITED, &mut |_| {})[0];
  assert_eq!(lost(&mut link, 1_000 + TIMEOUT_MS), None);
  assert_eq!(lost(&mut link, 1_000 + TIMEOUT_MS + 1), Some(Change::Lost));
  assert!(!link.present(MIRROR_1));
  assert_eq!(lost(&mut link, 1_000 + TIMEOUT_MS + 2), None);
}

#[test]
fn the_last_status_of_each_channel_is_kept_for_each_node() {
  let mut link = link();
  assert_eq!(link.on_frame(10, status(MIRROR_1, 0, 120)), None);
  link.on_frame(20, status(MIRROR_1, 0, 150));
  link.on_frame(30, status(MIRROR_1, 7, -40));
  link.on_frame(40, status(NodeId::MIRROR_2, 0, 9));
  let heard = link.heard(MIRROR_1).unwrap();
  assert_eq!(heard.status[0].map(|status| status.pos_steps), Some(150));
  assert_eq!(heard.status[7].map(|status| status.pos_steps), Some(-40));
  assert!(heard.status[1..7].iter().all(Option::is_none));
  let other = link.heard(NodeId::MIRROR_2).unwrap();
  assert_eq!(other.status[0].map(|status| status.pos_steps), Some(9));
}

#[test]
fn frames_in_the_coordinators_name_are_never_taken_for_a_node() {
  let mut link = link();
  assert_eq!(link.on_frame(10, heartbeat(10, 0)), None);
  assert_eq!(link.on_frame(10, status(NodeId::COORDINATOR, 0, 5)), None);
  assert!(WATCHED.iter().all(|node| !link.present(*node)));
  assert!(WATCHED.iter().all(|node| link.heard(*node) == Some(&Heard::default())));
}

#[test]
fn a_node_that_is_not_watched_changes_nothing() {
  let mut link = link();
  let stranger = NodeId::new(9).unwrap();
  assert_eq!(link.on_frame(10, node_beat(stranger, 10, 1)), None);
  assert_eq!(link.on_frame(10, status(stranger, 0, 5)), None);
  assert_eq!(link.heard(stranger), None);
  assert!(!link.present(stranger));
}

#[test]
fn a_heartbeat_of_another_protocol_version_is_reported_and_the_node_is_not_present() {
  let mut link = link();
  let mut beat = node_beat(MIRROR_1, 10, 1);
  if let Message::Heartbeat { proto_version, .. } = &mut beat.message {
    *proto_version = PROTOCOL_VERSION.wrapping_add(1);
  }
  assert_eq!(
    link.on_frame(10, beat),
    Some((
      MIRROR_1,
      Change::ProtocolMismatch {
        version: PROTOCOL_VERSION.wrapping_add(1)
      }
    ))
  );
  assert!(!link.present(MIRROR_1));
}
