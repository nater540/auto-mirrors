//! The watch over the nodes, one rule at a time, and a property over any order of heartbeats and ticks.

extern crate std;

use std::vec::Vec;

use proptest::prelude::*;

use super::*;

const TIMING: NodeTiming = NodeTiming::STARTING;
const TIMEOUT_MS: u64 = TIMING.heartbeat_timeout_ms as u64;
const MIRROR_1: NodeId = NodeId::MIRROR_1;

fn nodes() -> Nodes {
  Nodes::new(&TIMING)
}

/// A heartbeat of this protocol version.
fn beat(nodes: &mut Nodes, now_ms: u64, node: NodeId, uptime_ms: u32) -> Option<Change> {
  nodes.on_heartbeat(now_ms, node, uptime_ms, PROTOCOL_VERSION)
}

/// What `tick` reported for mirror 1.
fn lost_mirror_1(nodes: &mut Nodes, now_ms: u64) -> Option<Change> {
  nodes.tick(now_ms)[0]
}

#[test]
fn the_first_heartbeat_makes_a_node_appear_and_the_next_ones_change_nothing() {
  let mut nodes = nodes();
  assert!(!nodes.present(MIRROR_1));
  assert_eq!(beat(&mut nodes, 1_000, MIRROR_1, 5_000), Some(Change::Appeared));
  assert_eq!(beat(&mut nodes, 1_100, MIRROR_1, 5_100), None);
  assert!(nodes.present(MIRROR_1));
}

#[test]
fn a_node_is_lost_once_its_heartbeat_is_older_than_the_timeout_and_reported_once() {
  let mut nodes = nodes();
  beat(&mut nodes, 1_000, MIRROR_1, 5_000);
  assert_eq!(
    lost_mirror_1(&mut nodes, 1_000 + TIMEOUT_MS),
    None,
    "exactly the timeout is not yet a loss"
  );
  assert!(nodes.present(MIRROR_1));
  assert_eq!(lost_mirror_1(&mut nodes, 1_001 + TIMEOUT_MS), Some(Change::Lost));
  assert!(!nodes.present(MIRROR_1));
  assert_eq!(lost_mirror_1(&mut nodes, 5_000), None, "reported once");
}

#[test]
fn a_node_never_seen_is_never_lost() {
  let mut nodes = nodes();
  assert_eq!(nodes.tick(1_000_000), [None; 3]);
}

#[test]
fn a_lost_node_that_kept_running_appears_again() {
  let mut nodes = nodes();
  beat(&mut nodes, 1_000, MIRROR_1, 5_000);
  lost_mirror_1(&mut nodes, 2_000);
  // Ten seconds of silence, ten seconds more uptime: the bus was down, not the node.
  assert_eq!(beat(&mut nodes, 11_000, MIRROR_1, 15_000), Some(Change::Appeared));
}

#[test]
fn a_lost_node_that_restarted_meanwhile_is_reported_as_restarted() {
  let mut nodes = nodes();
  beat(&mut nodes, 1_000, MIRROR_1, 5_000);
  lost_mirror_1(&mut nodes, 2_000);
  assert_eq!(beat(&mut nodes, 11_000, MIRROR_1, 400), Some(Change::Restarted));
  assert!(nodes.present(MIRROR_1));
}

#[test]
fn a_restart_inside_the_timeout_is_seen_from_the_uptime() {
  let mut nodes = nodes();
  beat(&mut nodes, 1_000, MIRROR_1, 90_000);
  assert_eq!(beat(&mut nodes, 1_200, MIRROR_1, 150), Some(Change::Restarted));
  assert_eq!(beat(&mut nodes, 1_300, MIRROR_1, 250), None, "and once only");
}

#[test]
fn an_uptime_that_disagrees_by_exactly_the_timeout_is_not_a_restart() {
  let timeout_ms = u32::try_from(TIMEOUT_MS).unwrap();
  let mut nodes = nodes();
  beat(&mut nodes, 1_000, MIRROR_1, 5_000);
  assert_eq!(beat(&mut nodes, 1_100, MIRROR_1, 5_100 + timeout_ms), None);
  assert_eq!(
    beat(&mut nodes, 1_200, MIRROR_1, 5_200 + 2 * timeout_ms + 1),
    Some(Change::Restarted)
  );
  // Behind the clock counts the same way.
  let mut nodes = Nodes::new(&TIMING);
  beat(&mut nodes, 1_000, MIRROR_1, 5_000);
  assert_eq!(beat(&mut nodes, 1_100 + TIMEOUT_MS, MIRROR_1, 5_100), None);
  assert_eq!(
    beat(&mut nodes, 1_201 + 2 * TIMEOUT_MS, MIRROR_1, 5_200),
    Some(Change::Restarted)
  );
}

#[test]
fn the_wrap_of_the_uptime_after_49_days_is_not_a_restart() {
  let mut nodes = nodes();
  beat(&mut nodes, 1_000, MIRROR_1, u32::MAX - 50);
  assert_eq!(beat(&mut nodes, 1_100, MIRROR_1, 49), None);
}

#[test]
fn another_protocol_version_is_reported_once_and_does_not_count_as_present() {
  let mut nodes = nodes();
  beat(&mut nodes, 1_000, MIRROR_1, 5_000);
  let old = PROTOCOL_VERSION - 1;
  assert_eq!(
    nodes.on_heartbeat(1_100, MIRROR_1, 5_100, old),
    Some(Change::ProtocolMismatch { version: old })
  );
  assert!(!nodes.present(MIRROR_1));
  assert_eq!(
    nodes.on_heartbeat(1_200, MIRROR_1, 5_200, old),
    None,
    "once for each version"
  );
  let newer = PROTOCOL_VERSION + 1;
  assert_eq!(
    nodes.on_heartbeat(1_300, MIRROR_1, 5_300, newer),
    Some(Change::ProtocolMismatch { version: newer })
  );
  assert_eq!(
    nodes.tick(10_000),
    [None; 3],
    "a node that is not present is not lost again"
  );
  // Flashed back to this version: it appears, its old uptime forgotten.
  assert_eq!(beat(&mut nodes, 20_000, MIRROR_1, 300), Some(Change::Appeared));
  assert_eq!(
    nodes.on_heartbeat(20_100, MIRROR_1, 400, old),
    Some(Change::ProtocolMismatch { version: old }),
    "a version reported before is reported again after a match"
  );
}

#[test]
fn each_node_is_watched_on_its_own_and_unwatched_ones_change_nothing() {
  let mut nodes = nodes();
  for (index, node) in WATCHED.into_iter().enumerate() {
    let uptime_ms = u32::try_from(index).unwrap() * 1_000;
    assert_eq!(beat(&mut nodes, 1_000, node, uptime_ms), Some(Change::Appeared));
  }
  assert!(nodes.all_present());
  beat(&mut nodes, 1_250, NodeId::MIRROR_2, 1_250);
  assert_eq!(nodes.tick(1_400), [Some(Change::Lost), None, Some(Change::Lost)]);
  assert!(!nodes.all_present());
  assert!(nodes.present(NodeId::MIRROR_2));
  for unwatched in [
    NodeId::COORDINATOR,
    NodeId::new(4).unwrap(),
    NodeId::new(NodeId::MAX).unwrap()
  ] {
    assert_eq!(beat(&mut nodes, 1_500, unwatched, 0), None);
    assert!(!nodes.present(unwatched));
  }
}

#[test]
fn a_clock_that_seems_to_go_back_neither_loses_a_node_nor_panics() {
  let mut nodes = nodes();
  beat(&mut nodes, 5_000, MIRROR_1, 5_000);
  assert_eq!(lost_mirror_1(&mut nodes, 0), None);
  assert_eq!(beat(&mut nodes, 4_000, MIRROR_1, 5_100), None);
}

#[derive(Clone, Debug)]
enum Input {
  Beat {
    after_ms: u16,
    node: u8,
    uptime_ms: u32,
    proto_version: u8
  },
  Tick {
    after_ms: u16
  }
}

fn any_input() -> impl Strategy<Value = Input> {
  prop_oneof![
    (
      any::<u16>(),
      0_u8..5,
      any::<u32>(),
      prop_oneof![Just(PROTOCOL_VERSION), any::<u8>()]
    )
      .prop_map(|(after_ms, node, uptime_ms, proto_version)| Input::Beat {
        after_ms,
        node,
        uptime_ms,
        proto_version
      }),
    any::<u16>().prop_map(|after_ms| Input::Tick { after_ms })
  ]
}

proptest! {
  /// Whatever arrives, a node is lost only while present and present only after a heartbeat of this version, so
  /// that every Lost follows an Appeared or a Restarted, and the answers of `present` agree with the changes.
  #[test]
  fn changes_alternate_between_present_and_gone(inputs in proptest::collection::vec(any_input(), 0..200)) {
    let mut nodes = nodes();
    let mut now_ms = 0_u64;
    let mut present = [false; 3];
    let mut changes: Vec<(usize, Change)> = Vec::new();
    for input in inputs {
      match input {
        Input::Beat { after_ms, node, uptime_ms, proto_version } => {
          now_ms += u64::from(after_ms);
          let node = NodeId::new(node).unwrap();
          if let Some(change) = nodes.on_heartbeat(now_ms, node, uptime_ms, proto_version)
            && let Some(index) = WATCHED.iter().position(|watched| *watched == node)
          {
            changes.push((index, change));
          }
        }
        Input::Tick { after_ms } => {
          now_ms += u64::from(after_ms);
          for (index, change) in nodes.tick(now_ms).into_iter().enumerate() {
            changes.extend(change.map(|change| (index, change)));
          }
        }
      }
      for (index, change) in changes.drain(..) {
        match change {
          Change::Appeared => prop_assert!(!present[index], "appeared while present"),
          Change::Lost => prop_assert!(present[index], "lost while not present"),
          Change::Restarted | Change::ProtocolMismatch { .. } => {}
        }
        present[index] = matches!(change, Change::Appeared | Change::Restarted);
      }
      for (index, node) in WATCHED.into_iter().enumerate() {
        prop_assert_eq!(nodes.present(node), present[index]);
      }
    }
  }
}
