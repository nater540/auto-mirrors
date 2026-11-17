# auto-mirrors

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

A closed-loop system that aligns the two steering mirrors of a flying-optics CO2 laser without manual screw turning.
Four 28BYJ-48 steppers turn adjuster screws (two per mirror), a quadrant beam sensor at the head reports beam
position over CAN, and an ESP32-C6 coordinator computes corrections with an empirical Jacobian and damped least
squares. The Ruida RDC6445G keeps full control of gantry motion and laser firing: this system never fires the laser.

Hardware is still being designed. The work so far is host-testable logic. The development board on the bench is an
Adafruit Feather ESP32-C6 (no relay or bus attached); the owner allowed Claude to flash it on 2026-11-15.
It is not an Espressif DevKitC-1: GPIO 10, 11, and 20 are not on its headers, and 18 to 20 serve its I2C port.

## Read first

| File | What it is |
|---|---|
| `docs/HANDOFF.md` | The owner's hand-off: settled decisions, safety rules, working agreement, milestones, open items |
| `docs/ADDENDUM_A_28BYJ48.md` | Stepper actuator, Series E mount geometry, constants |
| `docs/REFERENCE.md` | Optics and sensor math, solver, esp-hal notes, Ruida protocol and I/O, safety logic |
| `docs/VERSIONS.md` | Pinned crate versions, why, and what contradicted the reference |
| `docs/SENSOR_PLAN.md` | DRAFT plan for the sensor node firmware (milestones S0 to S4), awaiting the owner's sign-off |

**Precedence when they disagree: the hand-off, then the addendum, then the reference.** The hand-off also wins over
the house style below.

Facts in the documents carry evidence tags: VERIFIED, CALCULATED, UNVERIFIED, AS MODELED (and ASSUMED, FIRM).
**Never promote a value that is not VERIFIED or CALCULATED to a literal.** It goes in `align_core::config` with its
tag in the doc comment. Do not replace a measured-later value with an invented one: leave the tagged placeholder and
add it to the open list in the report.

## Milestone status

Stop at the end of each milestone and report to the owner (hand-off section 6).

- M0 scaffold: done. M1 core math: done. M2 simulator and closed loop: done. M3 protocol: done.
- M4 stepper and node firmware: code done and running on the development board with PROVISIONAL pins. Motor bench
  test on 2026-11-17 (a 28BYJ-48 on a ULN2003, wired as the node's `FLASH_NOTICE.txt` says, a 1 degree dial on the
  shaft): plus steps turn the shaft counterclockwise seen from its end, and the unit is a true 64:1 (2048 steps a
  turn, not the configured 2037.8864). Four turns of 2048 steps each came back to the same tick, and 40 steps each
  way moved 7 ticks (7.03 predicted), so gear play unloaded is under half a degree. The first move after the shaft
  was turned by hand unpowered came up about 1 degree short of the rest, not yet explained.
  Pulling the rail jumper mid-move ended it as `RailLost`, pulling the permit jumper as `Stopped`, each with its
  steps counted and the record suspect; a move on channel 1 was refused; no coil LED lit during reset or boot.
  The torque margin test of addendum A.6 waits for the real mount.
- M5 coordinator: started. Host-tested: the state machine of reference 7.3 (`align_core::coordinator`, whose extra
  states await the owner's sign-off), the watch over the nodes, readings, moves, the coordinator's heartbeat and
  inhibit level, and all of it together (`coordinator_logic::orchestrator`). The firmware, on a second Feather
  ESP32-C6 that talks to the node through SN65HVD230 transceivers, runs the interlock: its console's `jog` arms the
  relay output, waits for its feedback, sends INHIBIT 0 and the MOVE_REL, and disarms. On the bench the relay
  output is the node's permit and the feedback a jumper (wired and flashed: not yet). Its `measure` pulses FootSW,
  sends a SHOT per burst, and collects the sensor's readings, and `calibrate`, `align`, and `poke` run the aligner
  through the same interlock (`coordinator_logic::requests`). Measurement and the procedure are host-tested only,
  through the whole-system simulation: no sensor board or Ruida is on the bench.
- M6 bring-up tools, M7 Ruida UDP (optional, read-only): not started.
- Sensor track (`docs/SENSOR_PLAN.md`): S0 protocol, S1 `sensor-logic`, and S2 `firmware/sensor` done, with five
  buttons instead of a detector (owner, 2026-11-15). S3, the detector's method, waits for the detector, which is
  likely a pyroelectric quad; do not build detector processing before then.

## Things that must not be done

- Do not send any Ruida command that moves an axis or fires the laser.
- Do not add a code path that energizes motors at boot, or without a received command and a present permit.
- Do not flash hardware, or ask the owner to flash, without stating what the image does to motors and I/O. Flash
  only through `just flash`, and only an image whose notice was read in the same session.
- Never write code that assumes firmware alone prevents motion during firing. Firmware checks are the last layer,
  behind the force-guided relay and the switched motor rail.
- Never use strapping pins GPIO4, 5, 8, 9, 15 for a safety signal. Every pin lives in one `board.rs` per firmware
  crate: `firmware/node` and `firmware/sensor` have one, PROVISIONAL for development boards; the coordinator has no
  pins yet (hand-off open item 7).
- Do not expand scope into sensor hardware design or mechanical design.

## Settled decisions (do not re-propose; build each behind a seam)

- Axis order everywhere: `q = [M1_H, M1_V, M2_H, M2_V]`, in full steps (`align_core::axis`).
- The control target is the three-position difference vector `m = [hB - hA, vB - vA, hC - hB, vC - vB]`. Absolute
  readings are a health metric only, never a target: a constant head offset cannot be removed by tilt.
- The Jacobian is measured by poking, never derived from geometry. The conversions in `align_core::units` are for
  planning and the simulator.
- Move, fire, read strictly alternate. Motors move only in the coordinator state `Moving`; a fire edge there is an
  immediate fault. Nodes stop and disarm after 300 ms without a heartbeat.
- Steppers have no stall or position feedback. A non-responding axis is found from the beam data
  (`solver::prediction_mismatch`).
- The actuator stays behind a trait so a DC gearmotor with an encoder can replace the stepper.
- Fixed-layout manual CAN encoding. No serde on the wire.
- The alignment procedure is `align_core::aligner::Aligner`, generic over the `Actuator` and `BeamSensor` traits
  (`align_core::rig`). Both traits are `async`, because a move or a measurement takes seconds on the machine. The
  simulator and the coordinator run the same procedure.
- Three adopted recommendations await the owner's sign-off (ESP32-C6, the three-position measurement, the
  force-guided relay as primary interlock). Build to them.

## What the simulator settled (M2)

These fill gaps in the documents. Each was found by a failing closed-loop test, is explained at the top of
`crates/align-core/src/aligner.rs`, and should not be undone without a simulator run that shows why.

- **A step is judged only if its predicted change is above the judging floor** (`calibration::judging_floor`:
  `judge_noise_sigmas` of the noise of the whole change `dm`, four readings to a component, over the mismatch
  fraction; 0.073 at the placeholder noise). Addendum A.5 as written reports a healthy axis as not responding,
  because corrections of a few steps are inside stick-slip and noise. The first floor, five sigmas of one
  measurement component (0.035), still named a healthy axis in one run of eight at the configured noise (review of
  2026-11-15). An axis is named only if its command clears the noise its apparent step carries, `norm(row_j(J^-1))`.
  The floor models sensor noise only, not stick-slip: at a characterized noise of 0.002 (floor 0.029), one run in
  300 with a two-step stick needs a recalibration.
- **Broyden updates are not stored.** They help within one alignment; kept across sessions, their noise gathered
  (a column 50% wrong in 479 of 1200 chained sessions, same review). The stored calibration changes only by poking.
- **Closed-loop tests that exercise the judging configure the simulated sensor's own noise** (`characterized()`,
  0.002). At the placeholder 0.005, most steps of an in-range misalignment are too small to judge, so a dead mirror
  1 axis is found by the recalibration's poke (`WeakColumn`) instead of as `AxisNotResponding`. The tests about the
  placeholder use `Config::default()`.
- **Suspect means an axis fell short, not that the prediction was off** (`solver::unresponsive_axis`). A Jacobian
  30% too small has a plain mismatch of 43%, close to the 50% threshold; a mismatch in which no axis took less than
  half of its command is a Jacobian error, and the Broyden update corrects it.
- **The damping adapts** (reference 3.2: times 3 after a poor step, divided by 3 after a good one). With it, an
  accepted calibration converges. The conditioning gate at 50 is applied to the measured Jacobian, whose condition
  number noise inflates: keep the model condition number near 10 or below (a Y span of 200 mm or more against an X
  span of 600 mm).
- **A reading outside the solver's linear range still drives a step**, but nothing is learned from that step.
- **Rates are tested as counts over fixed seeds**, not as properties over every seed. "Converges within the
  iteration cap" holds for every run only while the seat sticks for two steps or less.
- **The thresholds assume a CO2-sized beam.** The poke size, the step clamp, and the 0.02 tolerance are in
  normalized sensor units. On a 600 by 800 mm frame (`just sweep --y-spans 600 --x-span 800 --beam-radius R`),
  2000 runs all converge at a beam radius of 3 mm or more, 83 percent at 2 mm, and none at 1 mm or 0.5 mm: a poke
  of three times the backlash leaves a small beam's linear range, and 0.02 of a small beam is finer than the seats'
  stick-slip. A small test beam (a laser pointer) needs expanding to about 3 mm radius or more, or the thresholds
  need restating in physical units; the first is the recommendation (2026-11-15).
- **The node's count is the position.** `Actuator::move_rel` returns the `pos_steps` of the node's `MOVE_DONE`, and
  `Stopped` and `RailLost` carry it; the aligner adopts it, and a count that disagrees with the commands makes the
  axis suspect. A move with no answer marks the axis `lost`: it does not move again until the coordinator reads the
  node's STATUS and calls `Aligner::restore_position` (M5). Before, a lost answer and a retry ran a screw to twice
  its soft limit (review of 2026-11-15).
- **A step that grows the residual by more than noise is poor**, judged or not, coarse or clamped, and a suspect
  step that grew it is poor too: a stored column with the wrong sign otherwise walks the beam off the sensor. A
  calibration records the direction signs it was measured under and is not used under others.
- **A success is confirmed** by a second measurement with no move between; the mean of the two must be inside the
  tolerance as well. On one measurement, 9 runs in 1000 at the placeholder noise stopped with the true residual
  above 0.03.
- The absolute two-position target is kept out of the product and exists only in
  `crates/align-sim/tests/closed_loop.rs`, through the `Aligner::with_target` seam.
- **Through the whole system the aligner converges as it does against the model alone** (`just sweep --system`,
  2026-11-17): 1000 runs at each of 500, 200, 100, and 60 mm gave the same converged counts and faults, and one or
  two runs in 1000 differed in needing a recalibration, from the sensor's Q15 rounding. No frame was rejected. An
  alignment took about 71 s of simulated time, 46 s of it in measurements and 2 s in the settle time before them;
  that number rests on the UNVERIFIED placeholder job of `align_sim::system::Hardware` (2 s of gantry travel
  between dots).

## What the protocol settled (M3)

Reference 5.4 and addendum A.8 leave these open; `align-proto` decides them, and its tests hold the decisions.

- **Addresses:** coordinator 0, mirror 1 node 1, mirror 2 node 2, sensor node 3 (`NodeId`). The identifier's node
  field names the node at the far end from the coordinator; a message to or from every node carries 0.
- **The `axis` field is a channel on one node board (0 to 7), not a solver axis.** Which channel turns which screw
  is wiring: the coordinator maps `align_core::axis::Axis` to a node and channel (M5).
- **The identifier's axis field equals the payload's axis byte**, and is zero for a message without one; the
  decoder rejects anything else.
- **Decoding is strict where a value drives a move, lenient where it only reports.** A wrong length, an undefined
  class, MOVE_REL flag bit, move result, or config key, or an inhibit byte other than 0 or 1, is an error. Pads are
  written as zero and ignored. An unknown fault code is carried through (`FaultCode` is a newtype).
- **Not yet defined, carried as raw bytes:** stop and estop reasons, the inhibit source, and the heartbeat state.
  Give them meaning in the milestone that uses them.
- **Protocol version 2 (S0, 2026-11-15):** the sensor classes 0xA SHOT, 0xB SENSOR_RESEND, 0xC SENSOR_CMD, and
  0xD SENSOR_CMD_REPLY; SENSOR's status byte (result code, calibrated bit, bench bit); HEARTBEAT lengthened to 7
  bytes by `proto_version`. A node ignores a coordinator heartbeat of another version, reports FAULT 0x34 once, and
  refuses to act. Bump `align_proto::PROTOCOL_VERSION` with any layout change, and rebuild every image together.
  The layouts and their reasons are `docs/SENSOR_PLAN.md` section 3.
- **Duplicate moves (`MoveRelTracker`, one per node):** the last sequence number with identical content is answered
  (`AlreadyRunning`, or the same MOVE_DONE again) and never run twice; with different content it is refused
  (`SeqReused`); a new move while one runs is refused (`Busy`). The tracker is RAM only: after a reboot it accepts
  anything, so the node must restore its position before answering a move.

## What the node settled (M4)

`crates/node-logic/src/node.rs` opens with the full list; the ones that shape other milestones:

- **A node boots inhibited.** Only an INHIBIT 0 from the coordinator allows bus moves, and an ESTOP inhibits again.
  The coordinator (M5) must send INHIBIT 0 on entering `Moving` and INHIBIT 1 on leaving it.
- **A node needs each channel's soft limits and direction sign by CONFIG after every boot** (they are kept in RAM).
  The coordinator sends them when it sees a node's uptime restart in its heartbeat.
- **The node runs the standard approach itself** when a MOVE_REL has `finish_plus`, through
  `align_core::approach::plan`. The coordinator's actuator must then send a single MOVE_REL with `finish_plus`, not
  the legs the `Aligner` plans, or the overshoot runs twice. Decide this seam in M5.
- **The arm pulse runs only while a move runs**, in software, so that a hung CPU drops the rail.
- **Records are written only between moves**, by the task that runs the moves, so no flash write happens while a
  motor steps. They live on the `nvs` partition; unreadable contents are erased at boot, every channel then suspect.
- **Refusals are FAULT frames** with codes 0x20 to 0x2D (`align_proto::FaultCode`), the detail being the move's
  sequence number or the config key.
- **The bench console** (`just monitor node`, then `help`) drives motors only in bench mode with the permit present.
  On the bench there is no switched rail: the firmware checks are the only layer, which the flash notice says.
  Its `zero` makes a channel's present position zero and trusted (the commissioning position the soft limits
  count from), in bench mode with no coordinator heard; nothing moves.
- **After the 2026-11-15 nemesis review:** a move's coils are released by a guard held for the move, so a timed
  out or dropped move cannot leave them on, and the stepper counts its steps so a cut-off move still reports them.
  The record carries its direction sign (format 2) and is written suspect before every move. A silent or restarted
  coordinator is forgotten (inhibited, moves forgotten). An ESTOP latches until INHIBIT 0; bench moves need no
  coordinator on the bus. The TIMG1 watchdog resets the node after 1 s without feeding, so a panic cannot hold a coil.
- **On the development boards, TWAI runs through an SN65HVD230 transceiver** in normal mode, TX on GPIO 23 (MI)
  and RX on GPIO 1 (A0), on the node and the coordinator alike. A frame then needs a second node to acknowledge it:
  a board alone on the bus logs its failed sends once a second, and esp-hal aborts each one at error passive.
- **The Feather has clean header pins for one motor channel only** (GPIO 3, 2, 21, 22 for IN1 to IN4); channel 1 is
  unpopulated there (`board::CHANNELS`). Two channels on one board wait for the node board.

## What the coordinator settled (M5)

`crates/align-core/src/coordinator.rs` opens with the full list; the nemesis review of 2026-11-15 set these:

- **No input is assumed before it is reported.** The relay feedback and the Status line start unknown; the
  self-test passes only once the loop has read closed and the Status line has been read.
- **The relay energizes only while the loop reads closed, and motion needs a feedback reading taken after the
  energize.** A loop that still reads open after a release counts as armed: a fire or a job start then faults.
- **A fire is a burst with a start (`Fire`, which counts the shot) and an end (`FireEnded`, `burst_gap_ms` after its
  last edge).** The quiet time counts from the end; a move or a job is refused during a burst.
- **A job ends only on a Status low after a Status high.** A job whose Status line never rose faults
  (`StatusNeverHigh`) after the fire timeout, and no move can be armed meanwhile.
- **Readings may complete before the job ends** and are kept until it does. The firmware must call
  `coordinator_logic::readings::Readings::begin` when a job starts and send `ReadingsComplete` only from it, so a
  completion always belongs to the running job. `reading_timeout_ms` counts from the Status low, not from the last
  SHOT as `docs/SENSOR_PLAN.md` section 13 says: the more lenient of the two.
- **The aligner plans the standard approach; the node runs exactly what it is sent.** Every leg is one MOVE_REL with
  `finish_plus` clear (`coordinator_logic::moves`), so the overshoot never runs twice. A refusal FAULT (codes 0x20
  to 0x2D and 0x34, the move's sequence number as detail) ends the move as `MoveFault::Refused`.
- Time going backward, a move past `move_timeout_ms` (a backstop to the nodes' own), and a failed self-test in
  Idle each stop everything. `Coordinator::motion_allowed` gives the level for repeating INHIBIT to the nodes.

`crates/coordinator-logic/src/orchestrator.rs` opens with the rules of a move; these shape the firmware:

- **INHIBIT 0 goes out before the MOVE_REL, on the one transmit queue**, and a MOVE_REL is resent only while
  Moving. The inhibit level goes out with every heartbeat and at once when it changes.
- **The relay feedback reads high when the laser loop is open**, through the relay's normally open auxiliary
  contact, with a pull-down: a broken wire reads closed, so it can never arm a move. Status and fire sense are
  active low with pull-ups, as an optocoupler pulls them; their polarity on the Mod5 is UNVERIFIED (open item 9).
- **Every node that appears or restarts gets each of its axes' soft limits and direction sign by CONFIG.** Which
  channel turns which screw is `orchestrator::WIRING`, PROVISIONAL: channel 0 horizontal, channel 1 vertical.
- **Only the nodes the firmware expects gate the self-test and inhibit when lost** (the bench expects node 1 only).
- **The heartbeat's state byte is the state's place in `State`**, 0 (Inhibited) to 8 (Fault): `state_code`.
- **The coordinator's watchdog** resets the chip, and so releases the relay, if its 1 ms control loop stops.
- **FootSW and the relay drive are on the pins seen to drive no ULN2003 input on through reset** (GPIO 3 and 22 on
  the Feather): FootSW held for 100 ms starts a laser job. The bench check speaks to loads that need milliamperes
  (a relay driver, an optocoupler), not to weak pulls.
- **A measurement ends with the three readings only on `MeasurementReady`**; a sensor rejection is
  `SensorFault::Rejected`, and every other end (a dot missing, readings timed out, an unexpected job or fire, a
  fault, a lost node) is `SensorFault::NoReading`. A new measurement forgets the readings of the last.
- **The SHOT carries no edge age** while fire sense is sampled each millisecond; an edge interrupt can give one
  once the Mod5 scope check settles what a dot looks like.
- **SHOT goes out in the coordinator's name (node 0), to every node**, as `align_proto::Shot` defines it; SENSOR_RESEND
  is addressed to the sensor (node 3). The whole-system simulation found the orchestrator addressing SHOT to the
  sensor, which ignores that, while the orchestrator's own tests agreed with the mistake.

The review of the orchestrator on 2026-11-17 (four defects, each with the test that failed on it) set these:

- **A feedback reading confirms the relay only from a later millisecond than the energize**
  (`align_core::coordinator`): the firmware samples its inputs at the top of the millisecond and may energize later in
  it, so a reading stamped with the energize's own time was taken before it.
- **FootSW is let go as soon as the job no longer waits to start**, and no request is accepted while it is held:
  held on, it could start another job after one that ended at once, or pause a running one (reference 6.6).
- **The node of the move asked for or running counts as expected while it does**: its loss inhibits, whether or
  not the self-test needs it.
- **A FAULT that answers no command carries `align_proto::DETAIL_NO_COMMAND`** (0xFFFF) as its detail, never a
  sequence number: a node's failed flash write was read as the refusal of move 0.
- **Reference 7.3's faults are raised**: a soft limit or a lost rail in a MOVE_DONE, a motor node's PIN_FAULT,
  RAIL_LOST_DURING_MOVE, or MOVE_TIMEOUT, and the coordinator's own CAN bus-off each fault until `clear`.
- **ESTOP is repeated with every heartbeat while faulted**, and a node that refuses a move as NOT_CONFIGURED is sent
  its CONFIG again: a frame dropped from a full queue is otherwise never resent.
- **The coordinator's control loop logs nothing of what the orchestrator reports** (a `logger` task does), and fire
  sense latches its edges by interrupt, so neither a burst of log lines nor a short dot can hide a fire.
- Left as they are: a node loss found by the tick inhibits 1 ms after the beacon; a `stop` during the quiet time still
  energizes the relay later and disarms at once, with nothing moved.

The alignment procedure on the coordinator (`coordinator_logic::requests`, the firmware's `procedure` task):

- **The aligner asks one move or measurement at a time, and the control loop starts it when it can**: in Idle, and
  a measurement only `settle_ms` (200 ms, UNVERIFIED) after the last move ended; nothing else applies the settle
  time. A request the coordinator refuses, or that waits 60 s, fails at once as the aligner knows a failure: a move
  as `MoveFault::Refused` with code 0 (`COORDINATOR_REFUSED`, which no node sends), a measurement as NoReading.
- **The procedure starts from the nodes' STATUS** (`requests::starting_axes`): the aligner keeps its own state of an
  axis where the node's count agrees, takes the node's count with the last direction unknown where it differs, and
  holds an axis lost while its node has not reported it.
- **Any fault of the procedure faults the coordinator**, cleared by hand: reference 7.3 lists a stall and three
  invalid shots, and stopping on the others too means someone looks before the next run.
- **The whole-system simulation asks through the same `Requests`**, so its tests and `just sweep --system` cover
  the code the firmware runs.

## What the sensor settled (S1, S2)

- **Five buttons are the reading until the detector exists** (owner, 2026-11-15). No detector processing is built;
  the detector is likely a pyroelectric quad, and its method is planned and reviewed before it is built (S3).
- **Buttons connect to 3V3 with internal pull-downs** (active high), on GPIO 18 to 21 (A to D) and 10 (center);
  TWAI keeps GPIO 22 and 23. A broken wire reads as not pressed.
- **The position is taken when the SHOT arrives**; nothing held is centered; a rejected chord (opposite corners,
  three buttons or more) reports NoShot. Readings go out `reply_delay_ms` (200) after the SHOT, in shot order.
- **Bench mode** (console or SENSOR_CMD Bench) makes a chord fire a bench shot and ignores SHOT frames; outside it,
  a reading always needs a SHOT. Calibration commands answer Refused until a detector exists.
- **The console can do everything the bus and the buttons can** (`hold`, `press`, `shot`, `sim next`, `sim sum`,
  `sim delay`), so the sensor is testable over USB alone. A console shot is reported as a bench reading, so that it
  can neither block nor impersonate a coordinator shot; console-held buttons apply at once.
- **Readings go out strictly in shot order**, a later shot waiting behind an earlier one. SENSOR_RESEND serves the
  newest bus reading for a seq (bench readings are never kept), and a coordinator reboot forgets them all.
- **A coordinator reboot is an uptime that disagrees with the local clock** by more than the loss window, so the
  49.7-day wrap of the u32 uptime is not one. The coordinator must judge node reboots the same way (M5).
- **Each button debounces on its own timer**, so one chattering input cannot freeze the rest.

## Workspace layout

One Cargo workspace, edition 2024, Rust 1.96.0 pinned. Dependencies and lints are declared once in the root
`Cargo.toml`.

- `crates/align-core`: `no_std`, no HAL. Config, unit conversions, quadrant math, the measurement vector, the
  solver, the `Actuator` and `BeamSensor` traits, the approach planner and soft limits, calibration, and the
  `Aligner` procedure. Later: the coordinator state machine (a pure function of state and event).
- `crates/align-proto`: `no_std`. The identifier layout (`frame`), the ten messages and their encoding through
  `Envelope` (`message`), and the node-side check that runs a resent MOVE_REL once (`dedup`). Frames are the
  `embedded_can::Frame` trait, so firmware passes its esp-hal TWAI frames in directly.
- `crates/align-sim`: `std`. The model of the machine (`world`), the model behind the two traits (`rig`), the
  closed-loop tests (`tests/closed_loop.rs`), and the `align-sim` command-line runner. Every machine number in it
  is an UNVERIFIED placeholder and a scenario parameter, never configuration. `system` runs the whole system behind
  the same traits: the orchestrator, two `node-logic` nodes, and a `sensor-logic` node on a simulated bus (frames
  encoded and decoded), with the relay, the motors, and the Ruida simulated; `tests/system.rs` aligns through it.
- `crates/stepper`: `no_std`. 28BYJ-48 full-step driver over `embedded_hal::digital::OutputPin` and the async
  `DelayNs`: stop flag, rail check, coils released on drop.
- `crates/node-logic`: `no_std`. The motor node as a state machine (`node`, behind the `NodeIo` trait), its flash
  record layout (`record`), and the bench console commands (`console`). All host-tested.
- `firmware/node` (`align-node`): wires `node-logic` to TWAI, the steppers, the arm pulse, the permit and rail
  inputs, flash, and the USB console. Every pin is in `src/board.rs`, PROVISIONAL for an Adafruit Feather ESP32-C6.
- `crates/sensor-logic`: `no_std`. The sensor node as a state machine (`node`, behind `SensorIo`, generic over the
  `ShotMethod` that reads a shot: the buttons by default, the simulated beam in `align-sim`), the five bench
  buttons (`buttons`: debounce, chords, positions), and its console.
- `firmware/sensor` (`align-sensor`): wires `sensor-logic` to TWAI, five button inputs, and the USB console. Drives
  no output pin and writes no flash. Pins in `src/board.rs`, PROVISIONAL for a development board of its own; they
  assume a DevKitC-1 (GPIO 10 and 20 are not on a Feather's headers) and need a new table for the board it gets.
- `crates/coordinator-logic`: `no_std`. The coordinator's bus side, turning traffic into the state machine's events:
  so far the watch over the nodes (`nodes`: present, lost, restarted, or another protocol version, judged from
  heartbeats the way the nodes judge the coordinator), and the sensor's readings paired with a job's shots
  (`readings`: by sequence number, late ones asked for again with SENSOR_RESEND, the node's rejections as faults),
  and one move in flight (`moves`: MOVE_REL with `finish_plus` clear, MOVE_DONE or a refusal FAULT back, resent
  while late, per-node sequence numbers). `link` sends the coordinator's heartbeat and inhibit level and keeps what
  each node last said; `console` parses the coordinator's console.
  `orchestrator` runs all of it with the state machine: the self-test, the nodes configured, a move armed, sent,
  and disarmed, and a measurement started, shot, and read. `requests` carries the aligner's moves and measurements
  to it one at a time, and gives the aligner the nodes' counts before a procedure.
- `firmware/coordinator` (`align-coordinator`): the orchestrator on TWAI, the relay drive and its feedback, FootSW,
  the Status and fire sense inputs, the watchdog, the aligner in its own task, and the console (`status`, `jog`,
  `stop`, `measure`, `calibrate`, `align`, `poke`, `clear`).
  Pins in `src/board.rs`, PROVISIONAL for a Feather.

Dependency direction: firmware and `align-sim` depend on `align-core`, `align-proto`, `stepper`, `node-logic`,
`sensor-logic`, and `coordinator-logic`; `node-logic` depends on the first three, `sensor-logic` and
`coordinator-logic` on the first two; `align-core` depends on none of them.
HAL types never enter the `crates/` libraries.

The firmware crates are workspace members but not `default-members`: a plain `cargo build`, `cargo test`, or
`cargo clippy` at the root works on the host crates only. **`--workspace` tries to build esp-hal for the host and
fails.** Build firmware by package and target, through the justfile.

## Commands

`just` lists every recipe by group. The ones used most:

```bash
just ci                    # everything that must pass before a commit, cheapest first
just test                  # host tests; `just test align-core` for one crate
just clippy                # host crates and tests, warnings denied (`just lint` is the same)
just fmt                   # format everything (nightly rustfmt, for `trailing_comma`)
just doc                   # API docs with warnings denied
just check-no-std          # build the no_std libraries for the ESP32-C6 (proves no std, no alloc)
just firmware              # build both images in release mode; never flashes
just image node            # write the flashable image to target/images/ and print its size; touches no board
just ports                 # serial ports, the Espressif ones apart from the rest
just flash node            # print the notice, need it acknowledged, flash, then monitor or check the boot log
just boot-log coordinator  # reset the board and check that its boot log holds the coordinator's whole notice
just flash sensor          # the sensor image: five buttons instead of a detector; drives no output
just sim --seed 7          # one simulated alignment, every move and measurement
just sweep --runs 5000     # convergence against conditioning (the Y span); --x-span and --beam-radius for a frame
just sweep --system        # the same through the whole system (bus, nodes, interlock), with the simulated time
just sweep-margins         # how much stick-slip, noise, backlash, and step loss the thresholds tolerate
just deny                  # licenses, advisories, bans, and sources (deny.toml)
just outdated              # the pinned crates against crates.io
just doctor                # the tools the recipes need, and what is missing
just mutants -j 4          # mutation testing of the alignment logic, about 5 minutes
just check-commits <base>  # format, clippy, tests, and the no_std build at every commit after <base>
cargo test -p align-core <test_name>   # one test
```

After changing the aligner, the solver, or a threshold: run `just sweep` and `just sweep-margins` before and after
and compare the tables, run `just test-stress` once, and run `just mutants`. A surviving mutant gets a test that
states the boundary; it is excluded in `.cargo/mutants.toml` only if it cannot change behavior, with the reason.
Before handing a branch over, run `just check-commits <base>`: every commit must pass, not only the last.
After changing the protocol, a node's logic, or the orchestrator, `cargo test -p align-sim --test system` is the
test that holds the devices to each other; a unit test that encodes the same mistake on both sides cannot.

**The board recipes talk to hardware: `flash`, `boot-log`, `monitor`, `reset`, `board-info`, `ports`.** The flash
gate and the boot log check are in `scripts/board.sh`.

- `just flash node` (or `coordinator`) is the only way to flash. It prints the crate's `FLASH_NOTICE.txt` and needs
  it acknowledged: the crate name typed at a terminal, or `ALIGN_FLASH_ACK=<crate>:<hash>` without one. The hash is
  the start of the notice's SHA-256 and the recipe prints it after the notice, so an acknowledgement goes stale
  the moment the notice changes. Claude flashes this way, and only after reading the notice it acknowledges, and
  says in its reply what the image does to motors and I/O. There is no cargo runner, on purpose: `cargo run` must
  not flash around the notice.
- Without a terminal, `flash` ends by reading the boot log and failing unless every line of the notice appears
  and nothing panicked: that proves which image is running. `just boot-log <crate>` does the same check without
  flashing. The logs land in `target/boot-logs/`.
- With no port named, espflash chooses the port by its own rules: `ESPFLASH_PORT`, then the USB devices saved in
  its `espflash_ports.toml` (on this machine, the Espressif USB-Serial-JTAG, 303a:1001), then a prompt, which fails
  without a terminal rather than guessing. The two LG monitor ports on this machine do not get in the way. Do not
  write port selection in the scripts: the boot log check asks espflash which port it chose and reuses it.
- **`firmware/<crate>/FLASH_NOTICE.txt` states what the image does to motors and I/O.** The image logs it at boot.
  Change it in the same commit as any change to what the firmware drives; a stale notice is a safety defect.
- Do not run `monitor`, `reset`, or `board-info` on a port without knowing what is on it. Other boards are often
  attached to this machine.

## Code style: non-negotiable

- **Two-space indentation, exclusively.** Rust, TOML, YAML, Markdown, shell: everything except the `justfile`
  (four spaces). Never let rustfmt or an editor reformat to four.
- **Aim for ~120-character lines**, comments and Markdown included. Break where the text has a natural pause; do
  not pad lines out to the limit. Never leave a last line of a few words ("as suspect.") under a full one:
  end lines at sentence or clause boundaries instead, one sentence per line when sentences run long.
- **No trailing comma after the last item of a multi-line list**: `use` lists, struct fields and literals, enum
  variants, arguments, arrays, and TOML arrays. rustfmt enforces it for Rust through `trailing_comma = "Never"`,
  an unstable option, so **format with `just fmt` (nightly rustfmt), never a plain `cargo fmt`**, which puts the
  commas back.
- **No em-dashes or en-dashes, anywhere**: code, comments, string literals, docs, commit messages. Use a colon, a
  comma, parentheses, a plain hyphen, or two sentences.
- **No contractions** in docs, comments, commit messages, or messages to the owner ("do not", never the short form).
- **US English only** ("color", "behavior", "initialize", "canceled"), in identifiers, error strings, and prose.
- **Units go in identifier names**: `_steps`, `_um`, `_urad`, `_ms`, `_us`, `_norm`, `_pps`. The sensor's own axes
  are `h` and `v`, because X and Y name the gantry axes.
- **No `unwrap()`, `expect()`, `panic!`, `todo!()`, or unchecked indexing in non-test code.** Faults are values.
  Use `?`, `.get()`, iterators, and `checked_*`/`saturating_*` arithmetic. Watch for panics inside innocent calls
  (`f32::clamp` with reversed bounds, matrix indexing, an SVD without an iteration cap). Tests may unwrap and index.
- **Typed errors.** `thiserror` enums (without default features, so they work in `no_std`). Never
  `Box<dyn Error>` in a public API. An error variant carries enough context to be understood without reading the
  source. Handle an error at the layer that can do something about it.
- **Comments explain constraints and the non-obvious why**, not what the next line does. Every public item has a
  `///` doc comment, with `# Errors` where it can fail; every file opens with a `//!` header. A doc comment on a
  config field opens with its evidence tag.
- **No `println!`/`eprintln!` for diagnostics.** The `crates/` libraries do not log at all: they return values.
  Firmware logs through `log` (esp-println back end).
- Imports are grouped std or core, external crates, `crate::`, separated by blank lines.
- **Clippy-clean at `-D warnings`**, pedantic included. Suppress a lint with `#[expect(..., reason = "...")]`, never
  a bare `#[allow]`.
- `unsafe` is denied in the workspace lints. If firmware ever needs it, it gets a `// SAFETY:` comment stating the
  invariant.
- Produce complete, compilable code. No stubs in production paths.

## Testing: write tests first wherever possible

- Red, green, refactor. Logic lands with the test that failed before it existed. Never weaken a test to make it
  pass; if a test looks wrong, stop and say so.
- **All logic must compile and test on the host** with `cargo test`: no board, no network, no downloaded data.
- Test names are sentences stating the behavior (`an_axis_that_did_not_move_makes_the_step_suspect`).
- **Unit tests reproduce the worked numbers in the documents** (the addendum A.3 table, the reference 1.4
  conditioning table). When a document number and the code disagree, say so; do not adjust either silently.
- Cover the failure paths as well as the happy path: every fault variant has a test that produces it, including
  NaN and infinite inputs.
- A rule that must hold for every input is a `proptest` property, beside the example tests and not in place of
  them. Commit the `proptest-regressions/` files a failing property writes.
- Before every commit: `just ci`. Say plainly what was run and what was not.

## Dependencies

- The esp-hal family is pinned with `=` and moves together; see `docs/VERSIONS.md` for the upgrade procedure. **Do
  not trust esp-hal, esp-rtos, or Embassy API shapes from memory or from the snippets in the reference.** Read the
  pinned source under `~/.cargo/registry` and the changelog first.
- **Search for an existing crate before writing a type or helper by hand** (crates.io, `cargo info`, and what the
  pinned esp-hal family already pulls in), and say what was found. Prefer a well-maintained crate to hand-written
  infrastructure, and avoid crates that have been dormant for over a year (a stable trait crate in an active
  repository, such as `embedded-can`, is the exception). Do not add one for something that is a few correct lines.
- Turn off default features and enable only what is used; put a comment above a dependency saying why it is there
  when that is not obvious.
- No `alloc` in the `crates/` libraries that firmware uses. `just check-no-std` proves it.
- `Cargo.lock` is committed.

## Working in this repository

- Infer conventions from the existing code first, then from this file and `docs/`.
- **Reporting to the owner: lead with problems and disagreements, flag weak or missing evidence, and give one
  recommendation instead of a list of options.**
- When a request conflicts with something written here or in the hand-off, say so before proceeding; do not
  silently work around it.
- When a design decision is settled, record it here with the reason, so it is not proposed again.
- The open items the owner must supply are hand-off section 8. Ask for one when a milestone needs it; do not block
  earlier milestones on it.
- Never quote a number, a command, or an example that was not actually run.
- Secrets (keys, tokens, `.env` files) never enter the repository; see `.gitignore`.
- Small commits, one concern each. Commit subjects are short, plain sentences, optionally prefixed with the area
  they touch (`align-core: ...`, `firmware: ...`). No conventional-commit type tags.
- Keep this file current: update the milestone status, the layout, the commands, and the settled decisions in the
  same change that makes them true.
