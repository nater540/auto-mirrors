# Hand-off: Automatic Mirror Alignment for a DIY CO2 Laser

You are starting the firmware and tooling for this project from an empty repository. Read this file fully before writing code. Hardware is still in the design phase, so the first milestones are host-testable logic and a simulator, not flashing boards.

## 0. Read first

| File | Purpose | Status |
|---|---|---|
| `docs/REFERENCE.md` | Main fact reference: optics math, sensor math, solver, Embassy and esp-hal notes, Ruida protocol and I/O, safety logic | The owner must save the research report here. If it is missing, ask for it before Milestone 2 |
| `docs/ADDENDUM_A_28BYJ48.md` | Stepper actuator option, Series E mount geometry (section A.12), constants | Supersedes the N20, encoder, and DRV8876 parts of the reference |

Where this hand-off and the documents disagree, this hand-off wins, then the addendum, then the reference.

Facts in those documents are tagged VERIFIED, CALCULATED, UNVERIFIED, or AS MODELED. Never promote an UNVERIFIED value to a hard-coded assumption. Put it in config with the tag in a doc comment.

## 1. What is being built

A closed-loop system that aligns the two steering mirrors of a flying-optics CO2 laser without manual screw turning.

- Four motorized adjuster screws: two on mirror 1 (frame mounted), two on mirror 2 (rides the Y gantry). The third screw on each mount is a manual pivot.
- A quadrant beam sensor at the laser head entrance measures beam position. Sensor hardware is designed separately. Treat it as a CAN node that reports normalized X, Y, and sum.
- A coordinator computes screw corrections with an empirical Jacobian and damped least squares, over 2 to 4 iterations.
- The Ruida RDC6445G controller keeps full control of gantry motion and laser firing. This system never fires the laser itself in Version 1.

## 2. Problems and open risks (know these before designing)

1. **Three report recommendations are adopted here but the owner has not explicitly signed off.** They are: ESP32-C6 instead of C3, the three-position difference measurement (2.2 below), and a force-guided relay as the primary interlock. Build to them, and keep each one behind a clear seam so it can change.
2. **The stepper choice is conditional.** The 28BYJ-48 must pass the torque margin test (addendum A.6). Keep the actuator behind a trait so a DC gearmotor with encoder can replace it.
3. **No stall or position feedback on the steppers.** The solver must detect non-responding axes from the beam data (addendum A.5).
4. **esp-hal APIs are moving.** `esp-hal-embassy` was replaced by `esp-rtos` and `esp-wifi` by `esp-radio`. TWAI and ADC need the `unstable` feature. Check current crate docs and changelogs before writing any HAL code. Do not trust API shapes from memory or from the reference snippets. Pin exact versions.
5. **Sensor characteristics are unknown.** Response may be seconds per reading. Nothing may depend on fast feedback.
6. **Nothing about the Ruida UDP protocol is verified on the Mod5 firmware.** It is out of scope until Milestone 7 and is read-only even then.
7. **Mount seats are over-constrained** (three tips in three holes). Expect hysteresis. The simulator must model backlash and stick-slip so the solver is tested against them.
8. **Mirror 1 mount geometry is unverified.** Only the second mirror was checked against CAD.

## 3. Settled decisions

### 3.1 Hardware

| Item | Decision |
|---|---|
| MCU | ESP32-C6 for all nodes and the coordinator (stable Rust, `riscv32imac-unknown-none-elf`) |
| Actuators | 28BYJ-48 5 V geared steppers on ULN2003 drivers, full-step two-phase-on, one axis at a time per node, coils off between moves |
| Mounts | Cloudray Series E. Adjusters M6 x 0.5 (AS MODELED), lever arm 35 mm, L pattern |
| Bus | CAN (TWAI) at 250 kbit/s, SN65HVD230 transceivers. Five wires in the drag chain: power, ground, CAN H, CAN L, hardware motor permit |
| Nodes | Node 1 at mirror 1, node 2 at mirror 2, sensor node at the head, one coordinator. Node boards have three stepper channels, two populated |
| Ruida link, Version 1 | Hardware I/O only: start a canned job by pulsing FootSW, read CN1 Status for job running, read the laser fire line through an optocoupler |
| Motor power gate | A switched motor rail per node. `MOTOR_RAIL_EN = PERMIT_OK AND NOT(FIRE_RECENT) AND ARM_PULSE_OK`. The MCU contributes only a 1 kHz arm pulse through a charge pump |
| Primary interlock | Force-guided relay on the coordinator: motor permit asserted implies the laser enable loop is open |

Pin assignments do not exist yet. Put every pin in one `board.rs` per firmware crate. Never use strapping pins GPIO4, 5, 8, 9, 15 for safety signals.

### 3.2 Measurement and solver

- Axis order everywhere: `q = [M1_H, M1_V, M2_H, M2_V]`, in full steps.
- Three gantry positions: A (Y near, X near), B (Y far, X near), C (Y far, X far).
- Measurement vector: `m = [hB - hA, vB - vA, hC - hB, vC - vB]`, target zero. Keep `hA, vA` as a health metric only. Do not use absolute readings as the control target.
- Jacobian: measured empirically by poking each axis. 4x4, expected block-triangular.
- Step: `dq = alpha * (J^T J + lambda^2 I)^-1 J^T (-m)`, `alpha = 0.6`, `lambda = 0.05 * sigma_max(J)`.
- Broyden rank-one update after each accepted step.
- Converged at `norm_inf(m) <= 0.02`. At most 4 iterations, then recalibrate once, then fault.
- Reject a Jacobian with condition number above 50.
- Every final approach ends in the plus direction (screw advancing against the springs). Reverse moves overshoot by `OVERSHOOT_STEPS` first.
- After each step compare measured `dm` with `J * dq`. A large mismatch marks the axis suspect (addendum A.5).

All thresholds above are starting values. Make them config, not literals.

### 3.3 Safety rules for the firmware

- Move, fire, read strictly alternate. Motors may move only in the coordinator state `Moving`.
- A fire edge seen while `Moving` is an immediate fault.
- Nodes stop and disarm after 300 ms without a coordinator heartbeat.
- Position, phase index, and suspect flag persist to flash after every move, only while motors are stopped.
- Firmware checks are the last layer. Never write code that assumes firmware alone prevents motion during firing.

## 4. Working agreement

- Rust, `no_std` for firmware and core logic, Embassy for async.
- **Two-space indentation in every language.** Commit a `rustfmt.toml` containing `tab_spaces = 2`.
- US English. No em-dashes and no contractions in docs, comments, or messages to the owner.
- When reporting to the owner: lead with problems and disagreements, flag weak or missing evidence, and give one recommendation instead of a list of options.
- Units go in identifier names: `_steps`, `_um`, `_urad`, `_ms`, `_norm`.
- Core logic must compile and test on the host with `cargo test`. Keep HAL types out of it. Drive pins through `embedded_hal::digital::OutputPin`.
- No `unwrap` or `panic!` paths in firmware control flow. Faults are values.
- Fixed-layout manual CAN encoding. No serde on the wire.
- Small commits, one concern each. Run `cargo fmt`, `cargo clippy -- -D warnings`, and host tests before every commit.

## 5. Repository layout to create

```
Cargo.toml                 workspace
rustfmt.toml               tab_spaces = 2
rust-toolchain.toml        pinned stable
docs/                      reference documents (section 0)
crates/
  align-core/              no_std, no HAL: units, config, quadrant math, measurement vector,
                           Jacobian calibration, DLS solver, Broyden, approach planner,
                           coordinator state machine, validity checks
  align-proto/             no_std: CAN IDs, message structs, encode and decode
  align-sim/               std: optics and mount simulator, closed-loop tests, CLI runner
  stepper/                 no_std: 28BYJ-48 driver over embedded-hal, Actuator trait
firmware/
  node/                    ESP32-C6: stepper tasks, rail gate, TWAI, heartbeat, persistence
  coordinator/             ESP32-C6: orchestrator, Ruida I/O, relay, USB serial console
```

`align-core` owns an `Actuator` trait and a `BeamSensor` trait. The simulator and the firmware both implement them. The coordinator state machine is a pure function of (state, event) to (state, actions) so it can be tested exhaustively on the host.

## 6. Milestones

Stop at the end of each milestone and report to the owner.

**M0. Scaffold.** Workspace, toolchain pin, formatting, clippy, a CI script that runs host tests and builds firmware crates for `riscv32imac-unknown-none-elf`. Use `esp-generate` for the firmware crates after confirming its current options. Record pinned crate versions and why in `docs/VERSIONS.md`.

**M1. Core math.** In `align-core`: config struct with tagged defaults from addendum A.9 and A.12, step-to-beam conversions, quadrant normalization and validity checks, measurement vector from three readings, DLS step, Broyden update, condition number. Confirm that nalgebra fixed-size SVD and Cholesky build in `no_std` without alloc for the pinned version. If SVD does not, implement a small Jacobi SVD for 4x4. Unit tests reproduce the worked numbers in the documents.

**M2. Simulator and closed loop.** In `align-sim`: two-mirror geometric model (reference section 1.3), per-axis backlash, stick-slip, skipped steps, sensor noise, sensor linear range limit, constant head offset. Tests: converges from random misalignment within the iteration cap, converges with a 30% wrong Jacobian, detects a dead axis as suspect, never commands a move beyond soft limits, and a two-position absolute target fails where the three-position difference succeeds (this guards decision 3.2).

**M3. Protocol.** In `align-proto`: messages from reference section 5.4 with the field changes in addendum A.8. Round-trip tests for every message. Idempotent `MOVE_REL` by sequence number.

**M4. Stepper crate and node firmware.** Stepper driver per addendum A.4, including stop flag and rail-lost handling. Node firmware: TWAI receive and transmit with timeouts, heartbeat watchdog, one-axis-at-a-time moves, persistence, arm pulse output, rail sense. Include a bench mode that runs the torque margin test from addendum A.6 over the USB serial console.

**M5. Coordinator firmware.** State machine from reference section 7.3 wired to real I/O: FootSW pulse, Status input, fire sense input, relay drive with feedback check. A USB serial console for manual jog, poke, calibrate, align, and status dump.

**M6. Bring-up tools.** Console commands for backlash measurement and hysteresis characterization (reference section 3.4), logging in a format the owner can paste back for analysis.

**M7. Optional, later.** Ruida UDP, read-only: swizzle, checksum, firmware version query, position polling. No motion or fire commands until the owner confirms the read-only test passed on the Mod5.

## 7. Things you must not do

- Do not send any Ruida command that moves an axis or fires the laser.
- Do not add code paths that energize motors at boot or without a received command and a present permit.
- Do not flash hardware or ask the owner to flash without stating what the build will do to motors and I/O.
- Do not replace measured-later values with invented ones. Leave the tagged placeholder and add it to the open list.
- Do not expand scope into sensor hardware design or mechanical design.

## 8. Open items owned by the human

Ask for these when a milestone needs them. Do not block earlier milestones on them.

1. Torque margin test result and chosen step rate.
2. Coil resistance and steps per revolution of each motor.
3. 10-turn caliper check of the adjuster pitch.
4. Mirror 1 mount geometry.
5. Path lengths: mirror 1 to mirror 2 at Y near and far, mirror 2 to head at X near and far.
6. Sensor node interface details, time constant, noise, and beam radius at the head.
7. Pin assignments for the node and coordinator boards.
8. Laser power supply enable wiring for the interlock relay.
9. Ruida signal polarities (fire line, Status, FootSW) measured on the Mod5.
10. Sign-off on the three adopted recommendations in section 2, item 1.

## 9. First session checklist

1. Confirm the files in section 0 exist. Ask for `docs/REFERENCE.md` if it is missing.
2. Check current versions and docs for `esp-hal`, `esp-rtos`, `embassy-executor`, `embassy-time`, `embassy-sync`, `nalgebra`, and `esp-generate`. Write findings and pins to `docs/VERSIONS.md`. Report anything that contradicts the reference.
3. Complete M0 and M1.
4. Report: what was built, what contradicted the documents, and which open items now block progress.
