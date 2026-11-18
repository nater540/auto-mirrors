//! The probe against the simulated controller over real loopback sockets: what it reads, what it logs, and that it
//! sends nothing but the five reads whatever the controller does.

use std::net::UdpSocket;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use ruida_probe::probe::{self, POLLED, Settings, Summary};
use ruida_probe::sim::{self, Behavior, SimController};
use ruida_proto::{ChecksumBasis, Limits, Query};

/// Short enough that a silent controller costs the tests little.
const SETTINGS: Settings = Settings {
  limits: Limits {
    timeout_ms: 100,
    tries: 2
  },
  period_ms: 5,
  poll_ms: 0
};

/// Longer than any run here takes; a probe that has not finished by then never will.
const PROBE_DEADLINE: Duration = Duration::from_secs(10);

/// One probe run against `controller`: its summary, its log, and the controller as it ended. The probe runs in a
/// thread of its own, so a probe that never ends fails the test instead of hanging it.
#[expect(
  clippy::unwrap_used,
  clippy::expect_used,
  reason = "a test helper; a socket that cannot be bound on loopback, or a probe that hangs, fails the test"
)]
fn probe_against(controller: SimController, settings: &Settings) -> (Summary, Vec<String>, SimController) {
  let running = sim::serve(controller).unwrap();
  let address = running.address();
  let settings = *settings;
  let (sender, receiver) = mpsc::channel();
  thread::spawn(move || {
    let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    let mut log = Vec::new();
    let summary = probe::run(&socket, address, &settings, &mut log);
    // The test may have given up waiting; nothing is left to tell.
    let _ = sender.send(summary.map(|summary| (summary, log)));
  });
  let (summary, log) = receiver
    .recv_timeout(PROBE_DEADLINE)
    .expect("the probe did not finish")
    .unwrap();
  let controller = running.stop().unwrap();
  let log = String::from_utf8(log).unwrap().lines().map(str::to_owned).collect();
  (summary, log, controller)
}

fn controller(basis: ChecksumBasis, behavior: Behavior) -> SimController {
  let mut controller = SimController::new(basis);
  controller.behavior = behavior;
  controller
}

/// The log lines of one kind.
fn lines<'a>(log: &'a [String], kind: &str) -> Vec<&'a str> {
  let prefix = format!("RU1,{kind},");
  log
    .iter()
    .filter(|line| line.starts_with(&prefix))
    .map(String::as_str)
    .collect()
}

/// Every datagram the controller received was one of the five reads, at one basis or the other.
fn assert_only_reads_were_sent(controller: &SimController) {
  assert!(!controller.received_wire().is_empty());
  for datagram in controller.received_wire() {
    let is_a_read = Query::ALL.into_iter().any(|query| {
      [ChecksumBasis::Swizzled, ChecksumBasis::Plain]
        .into_iter()
        .any(|basis| query.packet(basis).bytes().as_slice() == datagram.as_slice())
    });
    assert!(
      is_a_read,
      "the probe sent {datagram:02x?}, which is not one of the five reads"
    );
  }
}

#[test]
fn the_probe_reads_the_five_values_of_a_controller_that_sums_the_swizzled_bytes() {
  let mut sim = SimController::new(ChecksumBasis::Swizzled);
  sim.status = 0x0100_0001;
  let (summary, log, sim) = probe_against(sim, &SETTINGS);
  assert_eq!(
    summary,
    Summary {
      answered: 5,
      failed: 0,
      basis: Some(ChecksumBasis::Swizzled),
      passed_over: 0,
      foreign: 0
    }
  );
  let readings = lines(&log, "reading");
  assert_eq!(readings.len(), 5);
  assert!(
    readings[0].contains(",query=card_id,card_id=0x65006500,basis=swizzled,sends=1"),
    "{}",
    readings[0]
  );
  assert!(
    readings[1].contains(",query=mainboard_version,mainboard_version=0x00030201,"),
    "{}",
    readings[1]
  );
  assert!(
    readings[2].contains(",query=status,status=0x01000001,moving=yes,part_end=no,job_running=yes,"),
    "{}",
    readings[2]
  );
  assert!(readings[3].contains(",query=x,x_um=125000,"), "{}", readings[3]);
  assert!(readings[4].contains(",query=y,y_um=-2500,"), "{}", readings[4]);
  let plain: Vec<Vec<u8>> = Query::ALL.into_iter().map(|query| query.plain().to_vec()).collect();
  assert_eq!(sim.received(), plain);
  assert_only_reads_were_sent(&sim);
}

#[test]
fn every_datagram_is_logged_with_its_wire_bytes_and_its_meaning() {
  let (_, log, _) = probe_against(SimController::new(ChecksumBasis::Swizzled), &SETTINGS);
  assert!(log[0].starts_with("RU1,start,controller=127.0.0.1:"), "{}", log[0]);
  assert!(
    log[0].contains(",magic=0x88,timeout_ms=100,tries=2,period_ms=5,poll_ms=0"),
    "{}",
    log[0]
  );
  let sends = lines(&log, "send");
  assert_eq!(sends.len(), 5);
  assert!(
    sends[0].ends_with(",query=card_id,try=1,basis=swizzled,wire=02 61 d4 89 0d f7,plain=da 00 05 7e"),
    "{}",
    sends[0]
  );
  let received = lines(&log, "recv");
  assert_eq!(received.len(), 10);
  assert!(
    received[0].ends_with(",wire=c6,plain=cc,meaning=ack"),
    "{}",
    received[0]
  );
  assert!(
    received[1].contains(",meaning=answer,address=05 7e,value=0x65006500,second=none"),
    "{}",
    received[1]
  );
  assert_eq!(
    log.last().unwrap(),
    "RU1,summary,answered=5,failed=0,basis=swizzled,passed_over=0,foreign=0"
  );
  assert!(log.iter().all(|line| line.starts_with("RU1,")));
}

#[test]
fn a_controller_that_sums_the_plain_bytes_is_found_by_its_first_nak_and_then_sent_plain() {
  let (summary, log, sim) = probe_against(SimController::new(ChecksumBasis::Plain), &SETTINGS);
  assert_eq!(summary.answered, 5);
  assert_eq!(summary.basis, Some(ChecksumBasis::Plain));
  let sends = lines(&log, "send");
  assert_eq!(sends.len(), 6);
  assert!(
    sends[0].contains(",query=card_id,try=1,basis=swizzled,"),
    "{}",
    sends[0]
  );
  assert!(sends[1].contains(",query=card_id,try=2,basis=plain,"), "{}", sends[1]);
  assert!(sends[2..].iter().all(|line| line.contains(",try=1,basis=plain,")));
  assert!(lines(&log, "reading")[0].ends_with(",basis=plain,sends=2"));
  assert_eq!(sim.received().len(), 6);
  assert_only_reads_were_sent(&sim);
}

#[test]
fn a_silent_controller_fails_every_query_once_each_without_sending_it_again() {
  let (summary, log, sim) = probe_against(controller(ChecksumBasis::Swizzled, Behavior::Silent), &SETTINGS);
  assert_eq!(
    summary,
    Summary {
      answered: 0,
      failed: 5,
      basis: None,
      passed_over: 0,
      foreign: 0
    }
  );
  assert!(
    lines(&log, "failed")
      .iter()
      .all(|line| line.ends_with(",reason=timed_out,sends=1"))
  );
  assert_eq!(sim.received().len(), 5);
  assert_only_reads_were_sent(&sim);
}

#[test]
fn a_controller_that_only_acks_shows_its_basis_but_gives_no_reading() {
  let (summary, log, _) = probe_against(controller(ChecksumBasis::Swizzled, Behavior::AckOnly), &SETTINGS);
  assert_eq!((summary.answered, summary.failed), (0, 5));
  assert_eq!(summary.basis, Some(ChecksumBasis::Swizzled));
  assert!(
    lines(&log, "failed")
      .iter()
      .all(|line| line.contains(",reason=timed_out_after_ack,"))
  );
}

#[test]
fn a_refusing_controller_fails_each_query_as_refused() {
  let (summary, log, _) = probe_against(controller(ChecksumBasis::Swizzled, Behavior::Refuse), &SETTINGS);
  assert_eq!((summary.answered, summary.failed, summary.basis), (0, 5, None));
  assert!(
    lines(&log, "failed")
      .iter()
      .all(|line| line.contains(",reason=refused,"))
  );
}

#[test]
fn a_stray_answer_is_logged_and_passed_over() {
  let (summary, log, _) = probe_against(controller(ChecksumBasis::Swizzled, Behavior::StrayFirst), &SETTINGS);
  assert_eq!(summary.answered, 5);
  assert_eq!(summary.passed_over, 5);
  let readings = lines(&log, "reading");
  assert!(
    readings[2].contains(",query=status,status=0x00000000,"),
    "{}",
    readings[2]
  );
  assert!(readings[3].contains(",query=x,x_um=125000,"), "{}", readings[3]);
}

#[test]
fn polling_reads_the_status_and_the_position_in_turn_after_the_five_reads() {
  let settings = Settings {
    poll_ms: 200,
    period_ms: 20,
    ..SETTINGS
  };
  let (summary, log, sim) = probe_against(SimController::new(ChecksumBasis::Swizzled), &settings);
  let polled: Vec<&str> = lines(&log, "reading")[5..]
    .iter()
    .map(|line| line.split(",query=").nth(1).unwrap().split(',').next().unwrap())
    .collect();
  // 200 ms at one query per 20 ms is about ten; a loaded machine may fit fewer.
  assert!((3..=11).contains(&polled.len()), "{polled:?}");
  for (index, name) in polled.iter().enumerate() {
    assert_eq!(*name, POLLED[index % 3].name());
  }
  assert_eq!(summary.failed, 0);
  assert_only_reads_were_sent(&sim);
}

#[test]
fn answers_with_a_second_value_are_read_by_their_first() {
  let mut sim = SimController::new(ChecksumBasis::Swizzled);
  sim.second_value = true;
  let (summary, log, _) = probe_against(sim, &SETTINGS);
  assert_eq!(summary.answered, 5);
  assert!(lines(&log, "recv")[1].contains(",value=0x65006500,second=0x65006500"));
}
