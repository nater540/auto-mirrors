//! The whole system on the host: the coordinator's orchestrator, the two mirror nodes, and the sensor node, each the
//! logic its firmware runs, exchanging encoded frames on a simulated bus over the simulated machine of
//! [`crate::world`].
//!
//! What stands in for the hardware, one millisecond at a time:
//!
//! - **The bus** delivers every frame to every other device one millisecond after it is sent, in the order sent.
//!   Each frame goes through `Envelope::encode` and `Envelope::decode`, as on TWAI. A test can have the next frame
//!   of a class dropped ([`System::drop_next`]): CAN retransmits on its own, so a frame is lost only where a queue
//!   overflows, and the protocol's resends are for that.
//! - **The relay:** its contacts follow the coordinator's drive after [`Hardware::relay_ms`]. The feedback contact
//!   goes to the coordinator; the motor permit goes to both nodes, and each node's rail is up while it is present.
//! - **The motors:** a node's move steps at its step period after its dwell, each step moving the world's screw by
//!   the true direction sign of its axis; a stop or a lost rail ends it at the steps issued so far.
//! - **The Ruida:** FootSW held active for 100 ms starts the canned job: Status up, a dot at each of A, B, and C,
//!   Status down ([`Hardware::job`]). The world is measured at the first dot; each dot shows the sensor its point.
//! - **The sensor** is the sensor node's own logic, with a method that reads the dot being fired.
//!
//! A test can also make things go wrong: a stray fire ([`System::fire_for`]), a job started at the Ruida's own panel
//! ([`System::start_job_at_ruida`]), or a coordinator that stops dead with its outputs where they were
//! ([`System::crash_coordinator`]), and see whether the nodes stop.
//!
//! [`SystemRig`] puts all of it behind the `Actuator` and `BeamSensor` traits of `align-core`, so the aligner that
//! will run on the coordinator runs here against the bus, the nodes, and the interlock.
//!
//! Every timing here (the relay, the job's dots) is an UNVERIFIED placeholder and a scenario parameter.

use std::cell::{Cell, Ref, RefCell, RefMut};
use std::collections::VecDeque;
use std::rc::Rc;

use align_core::aligner::{AlignFault, Aligner};
use align_core::axis::Axis;
use align_core::config::Config;
use align_core::coordinator::State;
use align_core::measurement::Shots;
use align_core::quadrant::Reading as BeamReading;
use align_core::rig::{Actuator, BeamSensor, MoveFault, SensorFault};
use align_proto::{Channel, Class, Envelope, NodeId, SUM_Q_ONE, SUM_Q_SATURATED, SensorResult};
use coordinator_logic::orchestrator::{CoordinatorIo, Inputs, Orchestrator, Report, RequestError, Setup, WIRING, Wire};
use coordinator_logic::requests::{COORDINATOR_REFUSED, Completion, Request, Requests};
use embedded_can::{Frame, Id, StandardId};
use node_logic::{MAX_CHANNELS, MoveOutcome, MoveRequest, Node, NodeIo, NodeSetup, Restored};
use sensor_logic::{ButtonBench, Reading as ShotReading, SensorIo, SensorNode, SensorSetup, ShotMethod};
use stepper::{MoveEnd, Phase};

use crate::world::{Scenario, World};

/// The nodes the coordinator's self-test needs: all three.
const EXPECTED: &[NodeId] = &[NodeId::MIRROR_1, NodeId::MIRROR_2, NodeId::SENSOR];

/// The motor nodes, in the order of [`System`]'s nodes.
const MOTOR_NODES: [NodeId; 2] = [NodeId::MIRROR_1, NodeId::MIRROR_2];

/// How long FootSW must be held for the Ruida to start the job (reference 6.6).
const FOOTSW_HOLD_MS: u64 = 100;

/// How long a request may wait for the coordinator to take it: the firmware's own limit.
const REQUEST_WAIT_MS: u32 = 60_000;

/// The canned job, in milliseconds from its start. UNVERIFIED placeholders: the gantry's travel between the points
/// sets the gaps on the machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JobTiming {
  /// Status goes up.
  pub status_up_ms: u64,
  /// The dots at A, B, and C start.
  pub dots_ms: [u64; 3],
  /// How long a dot fires.
  pub dot_ms: u64,
  /// Status goes down, and the job is over.
  pub end_ms: u64
}

/// The simulated hardware's timing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hardware {
  /// The relay's operate and release time.
  pub relay_ms: u64,
  /// The canned job.
  pub job: JobTiming
}

impl Hardware {
  /// UNVERIFIED placeholders: a 5 ms relay, and a job with 2 s of gantry travel between its dots.
  pub const PLACEHOLDER: Self = Self {
    relay_ms: 5,
    job: JobTiming {
      status_up_ms: 50,
      dots_ms: [500, 2_500, 4_500],
      dot_ms: 10,
      end_ms: 5_000
    }
  };
}

/// A CAN 2.0 data frame on the simulated bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SimFrame {
  id: StandardId,
  dlc: usize,
  data: [u8; 8]
}

impl Frame for SimFrame {
  fn new(id: impl Into<Id>, payload: &[u8]) -> Option<Self> {
    let Id::Standard(id) = id.into() else {
      return None;
    };
    let mut data = [0; 8];
    data.get_mut(..payload.len())?.copy_from_slice(payload);
    Some(Self {
      id,
      dlc: payload.len(),
      data
    })
  }

  fn new_remote(_id: impl Into<Id>, _dlc: usize) -> Option<Self> {
    None
  }

  fn is_extended(&self) -> bool {
    false
  }

  fn is_remote_frame(&self) -> bool {
    false
  }

  fn id(&self) -> Id {
    Id::Standard(self.id)
  }

  fn dlc(&self) -> usize {
    self.dlc
  }

  fn data(&self) -> &[u8] {
    self.data.get(..self.dlc).unwrap_or(&[])
  }
}

/// A device on the bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Device {
  Coordinator,
  Node(usize),
  Sensor
}

/// What happened on the bus, for the tests to check.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BusStats {
  /// Frames sent by any device.
  pub frames: u64,
  /// Frames no device could encode or decode: always zero unless the protocol and a device disagree.
  pub rejected: u64,
  /// Frames dropped on purpose ([`System::drop_next`]).
  pub dropped: u64
}

/// Collects what a node asks of its firmware during one call.
#[derive(Default)]
struct NodeOutbox {
  sent: Vec<Envelope>,
  start: Option<MoveRequest>,
  stop: bool,
  persisted: Vec<Channel>
}

impl NodeIo for NodeOutbox {
  fn send(&mut self, envelope: Envelope) {
    self.sent.push(envelope);
  }

  fn start_move(&mut self, request: MoveRequest) {
    self.start = Some(request);
  }

  fn stop_move(&mut self) {
    self.stop = true;
  }

  fn persist(&mut self, channel: Channel, _record: node_logic::AxisRecord) {
    self.persisted.push(channel);
  }
}

/// Collects what the coordinator asks of its firmware during one call.
#[derive(Default)]
struct CoordinatorOutbox {
  sent: Vec<Envelope>,
  relay: Option<bool>,
  footsw: Option<bool>,
  reports: Vec<Report>
}

impl CoordinatorIo for CoordinatorOutbox {
  fn send(&mut self, envelope: Envelope) {
    self.sent.push(envelope);
  }

  fn set_relay(&mut self, energized: bool) {
    self.relay = Some(energized);
  }

  fn set_footsw(&mut self, active: bool) {
    self.footsw = Some(active);
  }

  fn report(&mut self, report: Report) {
    self.reports.push(report);
  }
}

#[derive(Default)]
struct SensorOutbox {
  sent: Vec<Envelope>
}

impl SensorIo for SensorOutbox {
  fn send(&mut self, envelope: Envelope) {
    self.sent.push(envelope);
  }
}

/// What the sensor sees: the dot being fired, if any.
#[derive(Clone, Debug, Default)]
struct Dot(Rc<Cell<Option<ShotReading>>>);

impl ShotMethod for Dot {
  fn read(&self) -> ShotReading {
    self.0.get().unwrap_or(ShotReading {
      result: SensorResult::NoShot,
      h_q15: 0,
      v_q15: 0,
      sum_q: 0
    })
  }
}

/// A move a node's motor is running.
#[derive(Clone, Copy, Debug)]
struct Running {
  axis: Axis,
  request: MoveRequest,
  started_ms: u64,
  /// Physical steps issued so far, over both legs.
  issued: u32,
  /// The signed sum of the physical steps issued.
  issued_signed: i32,
  stop: bool
}

impl Running {
  fn total(&self) -> u32 {
    self.request.legs_steps.iter().map(|leg| leg.unsigned_abs()).sum()
  }

  /// The direction of physical step `index`: the first leg's steps, then the second's.
  fn direction(&self, index: u32) -> i32 {
    let [first, second] = self.request.legs_steps;
    if index < first.unsigned_abs() {
      first.signum()
    } else {
      second.signum()
    }
  }

  /// The steps due by `now_ms`: one per step period after the dwell.
  fn due(&self, now_ms: u64) -> u32 {
    let timing = &self.request.timing;
    let stepping_us = now_ms
      .saturating_sub(self.started_ms)
      .saturating_sub(u64::from(timing.energize_dwell_ms))
      .saturating_mul(1_000);
    let steps = stepping_us / u64::from(timing.step_period_us.max(1));
    u32::try_from(steps).unwrap_or(u32::MAX).min(self.total())
  }

  /// When the last step and the hold are over.
  fn done_ms(&self) -> u64 {
    let timing = &self.request.timing;
    let stepping_ms = u64::from(self.total()) * u64::from(timing.step_period_us) / 1_000;
    self.started_ms + u64::from(timing.energize_dwell_ms) + stepping_ms + u64::from(timing.hold_before_release_ms)
  }
}

/// The relay: what the coordinator drives, and where its contacts are.
#[derive(Clone, Copy, Debug, Default)]
struct Relay {
  drive: bool,
  /// When the drive last changed.
  changed_ms: u64,
  /// The contacts, which follow the drive after the operate time: the laser loop open and the motor permit given.
  contacts: bool
}

/// A motor node and its motor.
struct MotorNode {
  node: Node,
  running: Option<Running>
}

/// The coordinator, two mirror nodes, the sensor node, the bus, and the machine: see the module documentation.
pub struct System {
  hardware: Hardware,
  world: World,
  now_ms: u64,
  coordinator: Orchestrator,
  nodes: [MotorNode; 2],
  sensor: SensorNode<Dot>,
  dot: Dot,
  /// Frames in flight: when they arrive, who sent them, and the frame.
  bus: VecDeque<(u64, Device, SimFrame)>,
  stats: BusStats,
  relay: Relay,
  /// FootSW, and since when it has been active.
  footsw_since_ms: Option<u64>,
  /// The running job: when it started, and the readings its dots show.
  job: Option<(u64, Option<Shots>)>,
  /// The Ruida's outputs.
  status: bool,
  firing: bool,
  /// What the coordinator reported, with the time.
  reports: Vec<(u64, Report)>,
  /// Classes whose next frame is to be dropped, one entry per frame.
  drops: Vec<Class>,
  /// A stray fire, active until this time.
  stray_fire_until_ms: Option<u64>,
  /// The coordinator has stopped dead: it neither runs nor hears nor sends, and its outputs stay where they were.
  coordinator_down: bool,
  /// The procedure's requests, as the firmware's control loop keeps them, and the last completion.
  requests: Requests,
  completion: Option<Completion>,
  /// How many of the reports `requests` has seen.
  reports_seen: usize
}

impl System {
  /// The system after boot, on a machine in the scenario's starting state. The coordinator is configured with
  /// `config` and the machine's own mechanics, as the aligner will be.
  pub fn new(scenario: Scenario, mut config: Config, hardware: Hardware) -> Self {
    config.mechanics = scenario.mechanics;
    let coordinator = Orchestrator::new(Setup {
      config,
      fw_version: 1,
      expected: EXPECTED,
      wiring: WIRING
    });
    let nodes = MOTOR_NODES.map(|id| {
      let mut node = Node::new(NodeSetup {
        id,
        populated: 2,
        fw_version: 1,
        motion: config.motion,
        timing: config.node
      });
      node.restore([Restored::Missing; MAX_CHANNELS]);
      MotorNode { node, running: None }
    });
    let dot = Dot::default();
    let sensor = SensorNode::with_method(
      SensorSetup {
        fw_version: 1,
        timing: config.node,
        sensor_timing: config.sensor_timing,
        bench: ButtonBench::STARTING
      },
      dot.clone()
    );
    Self {
      hardware,
      world: World::new(scenario),
      now_ms: 0,
      coordinator,
      nodes,
      sensor,
      dot,
      bus: VecDeque::new(),
      stats: BusStats::default(),
      relay: Relay::default(),
      footsw_since_ms: None,
      job: None,
      status: false,
      firing: false,
      reports: Vec::new(),
      drops: Vec::new(),
      stray_fire_until_ms: None,
      coordinator_down: false,
      requests: Requests::new(config.motion.settle_ms, REQUEST_WAIT_MS),
      completion: None,
      reports_seen: 0
    }
  }

  /// Fires the laser for `duration_ms` from now, outside any job: a stray or manual fire.
  pub fn fire_for(&mut self, duration_ms: u64) {
    self.stray_fire_until_ms = Some(self.now_ms + duration_ms);
  }

  /// Starts the canned job as the Ruida's own start button would, without FootSW.
  pub fn start_job_at_ruida(&mut self) {
    if self.job.is_none() {
      self.job = Some((self.now_ms, None));
    }
  }

  /// Stops the coordinator dead, as a hang would: no tick, no frames in or out, and the relay drive left as it was.
  /// The firmware's watchdog would reset it and release the relay; this leaves the nodes on their own.
  pub fn crash_coordinator(&mut self) {
    self.coordinator_down = true;
  }

  /// The operator clears the coordinator's fault, as the console's `clear` does.
  pub fn clear_fault(&mut self) {
    let now = self.now_ms;
    self.with_coordinator(|coordinator, io| coordinator.clear_fault(now, io));
  }

  /// Asks the coordinator for a move and returns at once; the move runs as the system steps.
  ///
  /// # Errors
  ///
  /// The coordinator's [`RequestError`] when it refuses the move.
  pub fn start_move(&mut self, axis: Axis, delta_steps: i32) -> Result<(), RequestError> {
    let now = self.now_ms;
    self.with_coordinator(|coordinator, io| coordinator.request_move(now, axis, delta_steps, io))
  }

  /// Drops the next frame of `class` that any device sends, as a queue overflow would.
  pub fn drop_next(&mut self, class: Class) {
    self.drops.push(class);
  }

  /// The simulated time, in milliseconds since boot.
  pub const fn now_ms(&self) -> u64 {
    self.now_ms
  }

  /// The machine, to read its truth.
  pub const fn world(&self) -> &World {
    &self.world
  }

  /// The coordinator's state.
  pub const fn state(&self) -> State {
    self.coordinator.state()
  }

  /// What the coordinator reported so far, with the time of each.
  pub fn reports(&self) -> &[(u64, Report)] {
    &self.reports
  }

  /// What happened on the bus.
  pub const fn stats(&self) -> BusStats {
    self.stats
  }

  /// One millisecond.
  pub fn step(&mut self) {
    self.now_ms += 1;
    let now = self.now_ms;
    self.deliver(now);
    if now.saturating_sub(self.relay.changed_ms) >= self.hardware.relay_ms {
      self.relay.contacts = self.relay.drive;
    }
    self.run_ruida(now);
    for index in 0..self.nodes.len() {
      self.run_motor(now, index);
    }
    let stray_fire = self.stray_fire_until_ms.is_some_and(|until_ms| now < until_ms);
    let inputs = Inputs {
      loop_open: self.relay.contacts,
      job_running: self.status,
      firing: self.firing || stray_fire
    };
    if !self.coordinator_down {
      self.with_coordinator(|coordinator, io| coordinator.tick(now, inputs, io));
      self.run_requests(now);
    }
    let permit = self.relay.contacts;
    for index in 0..self.nodes.len() {
      self.with_node(index, |node, io| {
        node.set_permit(permit, io);
        node.set_rail_ok(permit);
        node.tick(now, io);
      });
    }
    let mut outbox = SensorOutbox::default();
    self.sensor.tick(now, &mut outbox);
    self.post(Device::Sensor, outbox.sent);
  }

  /// Runs for `duration_ms`.
  pub fn run_for(&mut self, duration_ms: u64) {
    for _ in 0..duration_ms {
      self.step();
    }
  }

  /// Runs until `done` holds, for at most `limit_ms`. Returns whether it held.
  pub fn run_until(&mut self, limit_ms: u64, mut done: impl FnMut(&Self) -> bool) -> bool {
    for _ in 0..limit_ms {
      if done(self) {
        return true;
      }
      self.step();
    }
    done(self)
  }

  /// Asks for the move through the coordinator's requests, as the firmware's procedure does, and runs until it
  /// completes.
  ///
  /// # Errors
  ///
  /// The move's [`MoveFault`]. One the coordinator could not start, or that did not complete within a minute, is
  /// [`MoveFault::Refused`] with code [`COORDINATOR_REFUSED`], or [`MoveFault::NoAnswer`].
  pub fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    match self.ask(Request::Move {
      axis,
      steps: delta_steps
    }) {
      Some(Completion::Move(result)) => result,
      Some(Completion::Measure(_)) => Err(MoveFault::NoAnswer),
      None => Err(MoveFault::Refused {
        code: COORDINATOR_REFUSED
      })
    }
  }

  /// Asks for a measurement through the coordinator's requests, as the firmware's procedure does, and runs until it
  /// completes.
  ///
  /// # Errors
  ///
  /// The measurement's [`SensorFault`]; one the coordinator could not start or end in time is
  /// [`SensorFault::NoReading`].
  pub fn measure(&mut self) -> Result<Shots, SensorFault> {
    match self.ask(Request::Measure) {
      Some(Completion::Measure(result)) => result,
      _ => Err(SensorFault::NoReading)
    }
  }

  /// Submits a request and runs until it completes, for at most a minute past its own wait limit.
  fn ask(&mut self, request: Request) -> Option<Completion> {
    const LIMIT_MS: u64 = 120_000;
    self.completion = None;
    self.requests.submit(self.now_ms, request).ok()?;
    self.run_until(LIMIT_MS, |system| system.completion.is_some());
    self.completion.take()
  }

  /// Starts the waiting request when the coordinator can take it, and finds its end among the reports made since
  /// the last step, as the firmware's control loop does.
  fn run_requests(&mut self, now: u64) {
    let mut requests = self.requests;
    let started = self.with_coordinator(|coordinator, io| requests.poll(now, coordinator, io));
    if let Some(completion) = started {
      self.completion = Some(completion);
    }
    for (_, report) in self.reports.get(self.reports_seen..).unwrap_or(&[]) {
      if let Some(completion) = requests.on_report(now, report) {
        self.completion = Some(completion);
      }
    }
    self.reports_seen = self.reports.len();
    self.requests = requests;
  }

  /// Delivers the frames due by `now` to every device but their sender.
  fn deliver(&mut self, now: u64) {
    while self.bus.front().is_some_and(|(due_ms, _, _)| *due_ms <= now) {
      let Some((_, from, frame)) = self.bus.pop_front() else {
        break;
      };
      let Ok(envelope) = Envelope::decode(&frame) else {
        self.stats.rejected += 1;
        continue;
      };
      if from != Device::Coordinator && !self.coordinator_down {
        self.with_coordinator(|coordinator, io| coordinator.on_frame(now, envelope, io));
      }
      for index in 0..self.nodes.len() {
        if from != Device::Node(index) {
          self.with_node(index, |node, io| node.on_frame(now, envelope, io));
        }
      }
      if from != Device::Sensor {
        let mut outbox = SensorOutbox::default();
        self.sensor.on_frame(now, envelope, &mut outbox);
        self.post(Device::Sensor, outbox.sent);
      }
    }
  }

  /// Puts frames on the bus, to arrive a millisecond from now.
  fn post(&mut self, from: Device, envelopes: Vec<Envelope>) {
    for envelope in envelopes {
      self.stats.frames += 1;
      if let Some(index) = self.drops.iter().position(|class| *class == envelope.message.class()) {
        self.drops.swap_remove(index);
        self.stats.dropped += 1;
        continue;
      }
      match envelope.encode::<SimFrame>() {
        Ok(frame) => self.bus.push_back((self.now_ms + 1, from, frame)),
        Err(_) => self.stats.rejected += 1
      }
    }
  }

  /// Calls the coordinator and carries out what it asked for.
  fn with_coordinator<R>(&mut self, call: impl FnOnce(&mut Orchestrator, &mut CoordinatorOutbox) -> R) -> R {
    let mut outbox = CoordinatorOutbox::default();
    let result = call(&mut self.coordinator, &mut outbox);
    if let Some(drive) = outbox.relay
      && drive != self.relay.drive
    {
      self.relay.drive = drive;
      self.relay.changed_ms = self.now_ms;
    }
    match outbox.footsw {
      Some(true) if self.footsw_since_ms.is_none() => self.footsw_since_ms = Some(self.now_ms),
      Some(false) => self.footsw_since_ms = None,
      _ => {}
    }
    let now = self.now_ms;
    self
      .reports
      .extend(outbox.reports.into_iter().map(|report| (now, report)));
    self.post(Device::Coordinator, outbox.sent);
    result
  }

  /// Calls a node and carries out what it asked for: frames, a move to start or stop, and records to write (each
  /// written at once).
  fn with_node(&mut self, index: usize, call: impl FnOnce(&mut Node, &mut NodeOutbox)) {
    let now = self.now_ms;
    let Some(motor) = self.nodes.get_mut(index) else {
      return;
    };
    let mut outbox = NodeOutbox::default();
    call(&mut motor.node, &mut outbox);
    let mut sent = core::mem::take(&mut outbox.sent);
    for channel in core::mem::take(&mut outbox.persisted) {
      let mut acknowledged = NodeOutbox::default();
      motor.node.on_persisted(channel, true, &mut acknowledged);
      sent.append(&mut acknowledged.sent);
    }
    if outbox.stop
      && let Some(running) = motor.running.as_mut()
    {
      running.stop = true;
    }
    if let Some(request) = outbox.start {
      let node_id = MOTOR_NODES.get(index).copied().unwrap_or(NodeId::MIRROR_1);
      let wire = Wire {
        node: node_id,
        channel: request.channel
      };
      if let Some(axis) = Axis::ALL.into_iter().find(|axis| WIRING.get(*axis) == wire) {
        motor.running = Some(Running {
          axis,
          request,
          started_ms: now,
          issued: 0,
          issued_signed: 0,
          stop: false
        });
      }
    }
    self.post(Device::Node(index), sent);
  }

  /// Steps a node's motor, and ends its move when it is done, stopped, or its rail is lost.
  fn run_motor(&mut self, now: u64, index: usize) {
    let rail = self.relay.contacts;
    let Some(running) = self.nodes.get_mut(index).and_then(|motor| motor.running.as_mut()) else {
      return;
    };
    let true_sign = self.world.scenario().mechanics.dir_sign.get(running.axis).factor();
    let cut = running.stop || !rail;
    let due = if cut { running.issued } else { running.due(now) };
    while running.issued < due {
      let direction = running.direction(running.issued);
      self.world.move_rel(running.axis, direction * true_sign);
      running.issued += 1;
      running.issued_signed += direction;
    }
    let end = if running.stop {
      Some(MoveEnd::Stopped)
    } else if !rail {
      Some(MoveEnd::RailLost)
    } else if now >= running.done_ms() {
      Some(MoveEnd::Done)
    } else {
      None
    };
    let Some(end) = end else {
      return;
    };
    let outcome = MoveOutcome::Finished {
      end,
      steps_issued: running.issued_signed,
      phase: Phase::ZERO
    };
    if let Some(motor) = self.nodes.get_mut(index) {
      motor.running = None;
    }
    self.with_node(index, |node, io| node.on_move_outcome(now, outcome, io));
  }

  /// The Ruida: a job started by FootSW, its Status line, and its dots.
  fn run_ruida(&mut self, now: u64) {
    let job_timing = self.hardware.job;
    if self.job.is_none()
      && self
        .footsw_since_ms
        .is_some_and(|since| now.saturating_sub(since) >= FOOTSW_HOLD_MS)
    {
      self.job = Some((now, None));
      self.footsw_since_ms = None;
    }
    let Some((started_ms, shots)) = self.job else {
      self.status = false;
      self.firing = false;
      self.dot.0.set(None);
      return;
    };
    let elapsed = now.saturating_sub(started_ms);
    if elapsed >= job_timing.end_ms {
      self.job = None;
      self.status = false;
      self.firing = false;
      self.dot.0.set(None);
      return;
    }
    self.status = elapsed >= job_timing.status_up_ms;
    let firing_dot = job_timing
      .dots_ms
      .iter()
      .position(|start| (*start..start + job_timing.dot_ms).contains(&elapsed));
    self.firing = firing_dot.is_some();
    if let Some(dot) = firing_dot {
      let shots = shots.unwrap_or_else(|| self.world.measure());
      self.job = Some((started_ms, Some(shots)));
      let reading = match dot {
        0 => shots.a,
        1 => shots.b,
        _ => shots.c
      };
      self
        .dot
        .0
        .set(Some(sensor_reading(&reading, self.world.scenario().beam.sum_ref)));
    }
  }
}

/// A reading of the world as the sensor reports it: Q15 positions, and the sum relative to the reference sum.
fn sensor_reading(reading: &BeamReading, sum_ref: f64) -> ShotReading {
  #[expect(
    clippy::cast_possible_truncation,
    reason = "a float to int `as` cast saturates, which clamps a position to the Q15 range"
  )]
  let q15 = |norm: f32| (f64::from(norm) * 32_768.0).round() as i16;
  let relative = f64::from(reading.sum) / sum_ref * f64::from(SUM_Q_ONE);
  #[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "clamped to the range of a reading that is not saturated"
  )]
  let sum_q = relative.round().clamp(0.0, f64::from(SUM_Q_SATURATED - 1)) as u16;
  ShotReading {
    result: SensorResult::Valid,
    h_q15: q15(reading.h_norm),
    v_q15: q15(reading.v_norm),
    sum_q
  }
}

/// A shared handle to one [`System`], behind the aligner's traits: clone it once for each role and keep a third to
/// look at the truth afterwards.
#[derive(Clone)]
pub struct SystemRig(Rc<RefCell<System>>);

impl SystemRig {
  /// The system, run until the coordinator has seen every node and passed its self-test, for at most 5 s.
  pub fn new(system: System) -> Self {
    let rig = Self(Rc::new(RefCell::new(system)));
    rig
      .system_mut()
      .run_until(5_000, |system| system.state() == State::Idle);
    rig
  }

  /// The system, to read it.
  ///
  /// # Panics
  ///
  /// If the system is mutably borrowed, which no method of this crate leaves it.
  pub fn system(&self) -> Ref<'_, System> {
    self.0.borrow()
  }

  /// The system, to change it.
  ///
  /// # Panics
  ///
  /// If the system is borrowed, which no method of this crate leaves it.
  pub fn system_mut(&self) -> RefMut<'_, System> {
    self.0.borrow_mut()
  }

  /// An aligner that drives this system. The sensor reports sums relative to its reference, so the aligner's
  /// reference sum is 1. The mechanics are the machine's own, as the coordinator's are.
  ///
  /// # Errors
  ///
  /// [`AlignFault::Config`] for a configuration the aligner refuses.
  pub fn aligner(&self, mut config: Config) -> Result<Aligner<Self, Self>, AlignFault> {
    config.mechanics = self.system().world().scenario().mechanics;
    Aligner::new(self.clone(), self.clone(), config, 1.0)
  }
}

impl Actuator for SystemRig {
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    self.system_mut().move_rel(axis, delta_steps)
  }
}

impl BeamSensor for SystemRig {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    self.system_mut().measure()
  }
}
