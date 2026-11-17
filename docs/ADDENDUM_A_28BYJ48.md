# Addendum A: 28BYJ-48 + ULN2003 Actuator Option

Companion to "Auto Mirror Alignment Reference: CO2 Laser, ESP32 + Embassy (Rust)". Section numbers below refer to that document.

Status: this option is CONDITIONAL on the torque margin test in A.6. If the test passes, this addendum replaces the N20, encoder, and DRV8876 material. If it fails, the base document stands unchanged.

Tags are the same as the base document: VERIFIED (named source), CALCULATED (formula and inputs shown), UNVERIFIED (must be measured or tested). Code uses two-space indentation.

---

## A.0 Problems first

| # | Problem | Severity | Status |
|---|---|---|---|
| A1 | "28BYJ-48" is a family, not one part. Gear ratios of 16:1, 64:1, and 63.68395:1 are all sold under the name, and coil resistance ranges from about 27 to 50 ohm on 5 V units. Identify each unit before use (A.6). | High | VERIFIED: splitflap motor buying guide; splitflap wiki; pcbsync |
| A2 | Torque margin over the thumbscrew is unknown. The conservative datasheet figure is only 34.3 mN·m. | High | UNVERIFIED: test A.6 |
| A3 | No stall or position feedback. Skipped steps are invisible to the node. Soft limits become approximate. | Medium | By design; mitigations in A.5 |
| A4 | Gearbox backlash is large and unpublished. Expect several degrees at the output. | Medium | UNVERIFIED: measure with base 3.4 |
| A5 | Steps per revolution is about 2038, not 2048, on the common variant. Never assume whole-revolution closure. | Low | VERIFIED: gear tooth count, pcbsync; splitflap wiki |
| A6 | No sleep pin. The interlock must gate the motor supply rail (A.7). | Medium | Design change |
| A7 | If the motor rail is cut mid-move, steps are lost while firmware keeps counting. | Medium | Mitigation in A.5 |

---

## A.1 What this replaces in the base document

| Base item | Change |
|---|---|
| Flag F1 (C3 has no PCNT) | Moot. No encoders. The ESP32-C3 is viable again. Keep the ESP32-C6 anyway: one target for nodes and coordinator, and room to return to N20 motors. |
| Flag F2 (DRV887x VM minimum) | Moot. No DRV887x. |
| 1.1 single-screw chain | Replace `counts / CPR / G` with `steps / STEPS_PER_REV` (A.3). |
| 3.1 poke size, 3.4 overshoot | New values in steps (A.3, A.5). Procedures unchanged. |
| 3.6 persistence | AxisPos gains `phase u8` and `suspect u8`. |
| 4.2 to 4.6 (encoder rates, DRV8876, current limit, move controller, ADC) | Replaced by A.2 to A.5. |
| 5.2 task table | axis task inputs become Command Channel and rail-sense pin only. No PCNT, ADC, or nFAULT. |
| 5.4 CAN protocol | Field meaning changes only (A.8). Layouts unchanged. |
| 7.1 node gate | Gate drives a motor rail switch instead of nSLEEP (A.7). |
| 8.1 N20 facts, 9.2 constants | Replaced by A.2 and A.9. |

Everything else (optics, sensor math, solver, Ruida, state machine) is unchanged.

---

## A.2 Motor and driver facts

| Item | Value | Status |
|---|---|---|
| Type | 4-phase unipolar permanent magnet stepper, 5 wires, geared | VERIFIED: Kiatronics datasheet |
| Rated voltage | 5 V DC | VERIFIED: Kiatronics datasheet |
| Stride angle | 5.625 deg / 64 (half-step, at output) | VERIFIED: Kiatronics datasheet |
| Motor steps per rotor rev | 32 full-step, 64 half-step | VERIFIED: crcibernetica; pcbsync |
| Nominal ratio | 1/64 | VERIFIED: Kiatronics datasheet |
| Actual ratio (common variant) | (32·22·32·22)/(9·10·9·10) = 63.68395 | VERIFIED: tooth count, pcbsync |
| Full steps per output rev | 32 × 63.68395 = 2037.8864 | CALCULATED; matches splitflap wiki measurement |
| Coil resistance | 50 ohm ±7% per datasheet; 27 to 35 ohm measured on many units | VERIFIED: Kiatronics, sgbotic datasheets; splitflap wiki; Solarbotics |
| Phase current at 5 V | 100 mA at 50 ohm; up to about 185 mA at 27 ohm, before driver drop | CALCULATED: V/R |
| Pull-in torque | > 34.3 mN·m (300 gf·cm) at 5 V, 100 to 120 pps | VERIFIED: Kiatronics, sgbotic datasheets |
| Vendor-measured torque | 821 gf·cm (80.5 mN·m) full-step at 200 mA; 598 gf·cm half-step | VERIFIED as vendor claim: RobotShop/Solarbotics |
| Self-positioning torque | > 34.3 mN·m | VERIFIED: Kiatronics datasheet |
| Friction torque | 600 to 1200 gf·cm (59 to 118 mN·m). Interpretation as unpowered back-drive resistance is UNVERIFIED | VERIFIED figure: Kiatronics datasheet |
| Max starting rate, no load | > 500 pps (sgbotic); > 600 Hz (Kiatronics) | VERIFIED; drive mode for these figures unstated |
| Size, mass | 28 mm diameter, 20 mm tall excluding shaft, about 30 g | VERIFIED: Solarbotics; canadarobotix |
| Shaft | 5 mm double-D | VERIFIED: RobotShop |
| Shaft offset from body center, ear hole spacing, flat width | About 8 mm, 35 mm, 3 mm | UNVERIFIED: measure your units |
| Wire colors | A blue, B pink, C yellow, D orange, common red | VERIFIED: canadarobotix |

ULN2003A (UNVERIFIED in this addendum; from memory of the TI datasheet, confirm before layout):

- Seven Darlington low-side sinks, 500 mA each, with clamp diodes to the COM pin.
- Saturation drop is roughly 0.9 to 1.1 V at these currents, so the coil sees about 4 V, not 5 V. Vendor torque figures taken with this driver already include that loss.
- COM must connect to the SWITCHED motor rail (A.7), not to an always-on 5 V.
- Inputs are driven directly from 3.3 V GPIO in common practice. Confirm against the input current curve.

---

## A.3 Step-to-beam math (CALCULATED)

```
screw_travel_m  = steps / STEPS_PER_REV * p
mirror_tilt_rad = screw_travel_m / L
beam_disp_m     = k * mirror_tilt_rad * D
=> beam_disp_m  = steps * p * k * D / (STEPS_PER_REV * L)
```

Worked example with Series E mount geometry from A.12 (p = 0.5 mm, L = 35 mm, k = 2, D = 1 m, STEPS_PER_REV = 2037.8864):

| Quantity | Full-step | Half-step |
|---|---|---|
| Screw travel per step | 0.245 um | 0.123 um |
| Mirror tilt per step | 7.01 urad | 3.50 urad |
| Beam displacement per step at 1 m, k = 2 | 14.0 um | 7.0 um |
| Beam displacement per step at 1 m, k = 1.414 | 9.9 um | 5.0 um |
| Steps per degree of screw | 5.66 | 11.32 |
| Steps per screw turn | 2038 | 4076 |

Poke size (base 3.1 formula, w = 4 mm): `0.25 / ((1.596 / 4000) * 14.0)` = about 45 full steps. Clamp to [3 × backlash_steps, 2000].

Speed at 200 full steps per second: 0.098 rev/s, 49 um/s of screw travel. A 500 step move takes 2.5 s.

Axial force bound, `F = 2*pi*eta*T / p`, p = 0.5 mm:

| Torque basis | T | F at eta 0.14 | F at eta 0.5 |
|---|---|---|---|
| Datasheet pull-in | 34.3 mN·m | 60 N | 216 N |
| Vendor full-step | 80.5 mN·m | 142 N | 506 N |

The eta values are the base document assumptions (UNVERIFIED). The stepper cannot exceed its pull-out torque, so this bound holds with no current limiting. That is the main protection advantage over the N20.

---

## A.4 Drive scheme (firm)

- **Full-step, two-phase-on.** Highest torque, uniform torque per state, four states. Half-step alternates weak one-phase states and buys resolution that is not needed. The datasheet prefers half-step; that is a disagreement, and torque margin wins here.
- **One axis at a time per node.** Caps node motor current at two phases (200 to 370 mA depending on coil variant) and keeps moves independent.
- **Coils off between moves.** Zero heat into the mount. Screw friction plus gearbox hold position.
- **Energize, dwell, step, hold, release.** Energize the stored phase state, wait 20 ms, step, hold the final state 100 ms, then all off.
- **Persist the phase index** with the position. On re-energize the rotor may jump by up to two motor steps (0.5 um of screw at most). That is far inside gearbox backlash and is absorbed by the standard approach.
- **No acceleration ramp** at or below 200 pps. Add a ramp only if a higher rate is wanted (UNVERIFIED; the test in A.6 decides).

| State | IN1 (blue) | IN2 (pink) | IN3 (yellow) | IN4 (orange) |
|---|---|---|---|---|
| 0 | 1 | 1 | 0 | 0 |
| 1 | 0 | 1 | 1 | 0 |
| 2 | 0 | 0 | 1 | 1 |
| 3 | 1 | 0 | 0 | 1 |

Which sequence direction advances the screw against the springs depends on wiring and thread hand: UNVERIFIED, set `DIR_SIGN` per axis at commissioning.

```rust
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::digital::{OutputPin, PinState};

const SEQ: [[bool; 4]; 4] = [
  [true, true, false, false],
  [false, true, true, false],
  [false, false, true, true],
  [true, false, false, true],
];

#[derive(Clone, Copy, PartialEq)]
pub enum MoveResult {
  Done,
  Stopped,
  RailLost,
}

pub struct Stepper<P: OutputPin> {
  pins: [P; 4],
  pub phase: u8,
  pub pos_steps: i32,
  pub suspect: bool,
}

impl<P: OutputPin> Stepper<P> {
  fn apply(&mut self, energized: bool) {
    let row = SEQ[(self.phase & 3) as usize];
    for (pin, on) in self.pins.iter_mut().zip(row) {
      let _ = pin.set_state(PinState::from(energized && on));
    }
  }

  /// Never drop this future mid-move: the coils would stay energized.
  /// Cancel through `stop` instead.
  pub async fn move_rel(
    &mut self,
    delta_steps: i32,
    step_period: Duration,
    stop: &AtomicBool,
    rail_ok: impl Fn() -> bool,
  ) -> MoveResult {
    let dir: i32 = if delta_steps >= 0 { 1 } else { -1 };
    let mut result = MoveResult::Done;
    self.apply(true);
    Timer::after(Duration::from_millis(20)).await;
    let mut ticker = Ticker::every(step_period);
    for _ in 0..delta_steps.unsigned_abs() {
      if stop.load(Ordering::Relaxed) {
        result = MoveResult::Stopped;
        break;
      }
      if !rail_ok() {
        self.suspect = true;
        result = MoveResult::RailLost;
        break;
      }
      self.phase = (self.phase as i32 + dir).rem_euclid(4) as u8;
      self.pos_steps += dir;
      self.apply(true);
      ticker.next().await;
    }
    Timer::after(Duration::from_millis(100)).await;
    self.apply(false);
    result
  }
}
```

The driver is generic over `embedded_hal::digital::OutputPin` so esp-hal pin API changes do not touch it. Signatures are UNVERIFIED against pinned crate versions.

---

## A.5 Logic changes without feedback

Stall detection (replaces base 4.5 stall rule): there is none at the node. Use these instead.

| Mechanism | Rule |
|---|---|
| Prediction check | After each solver step compare measured `dm` with `J * dq`. If `norm(dm - J*dq) > 0.5 * norm(J*dq)`, treat the move as suspect: skip the Broyden update, set `suspect`, and repeat the measurement. Threshold UNVERIFIED. |
| Two suspects in a row on one axis | Fault: "axis not responding" (skipping, cup slid off, or screw at end of travel). |
| Rail lost mid-move | Set `suspect`, report RailLost, do not trust `pos_steps` for soft limits until the next successful convergence. |
| Soft limits | Keep them, in steps, but widen by a margin equal to the largest plausible loss (start with 10% of travel). They are a runaway guard, not a position truth. |
| Per-move caps | At most 4000 steps (about 2 screw turns) and at most 25 s per move. UNVERIFIED starting values. |
| Mechanical fuse | Unchanged: the cup bottoms one way and slides off the other. |

Backlash and standard approach (base 3.4 procedure unchanged):

- Initial `OVERSHOOT_STEPS = 120` (about 21 degrees of screw). UNVERIFIED guess.
- After measuring backlash with the beam, set `OVERSHOOT_STEPS = max(2 × backlash_steps, 60)`.
- The backlash search sequence becomes b = 8, 16, 32, 64, 128 steps.

Settle time before a shot: 200 ms after coils release (UNVERIFIED; same as base).

---

## A.6 Identification and torque margin test (do this first)

Identify each unit:

1. Measure resistance from red to each coil wire. Expect one value near 27 to 50 ohm on a 5 V unit. A value near 200 ohm or more suggests a 12 V unit: set aside.
2. Mark the shaft and command 2038 full steps, four times. About 4 turns on the first command means a 16:1 unit: set aside. After the four, the common 63.68395:1 unit is back on the mark (within half a degree); a true 64:1 unit is about 7 degrees (40 steps) short of it, and commands of 2048 steps then bring it back to the mark every turn.
3. Use four matching units.

Torque margin, on the real mount with the real cup coupler and springs:

1. Mark the knob. Run +1000 then -1000 full steps, 10 cycles, at 100, 200, and 300 pps.
2. Repeat near both ends of the intended screw travel (spring force changes with position).
3. Repeat with the motor rail lowered to 4.5 V.
4. Pass: knob returns to the mark (within backlash) in every case at 4.5 V and 300 pps. Then run at 200 pps and 5 V in service.
5. Fail options, in order: clean and lubricate the screw threads; lower the rate to 100 pps; revert to the N20 design in the base document. A bipolar rewire with an H-bridge is widely reported to raise torque (UNVERIFIED) but removes the simplicity that justifies this option.

---

## A.7 Interlock and power changes (replaces the node gate in base 7.1)

```
MOTOR_RAIL_EN = PERMIT_OK AND NOT(FIRE_RECENT) AND ARM_PULSE_OK
```

- `MOTOR_RAIL_EN` drives a high-side switch (P-channel MOSFET or load switch) feeding the motor red wires and ULN2003 COM. Default off through a pull resistor.
- With the rail off, GPIO states cannot move a motor. A resetting or glitching MCU is harmless.
- Keep bulk capacitance on the switched side small (10 uF or less) and add a bleed resistor (about 1 kohm) so the rail collapses within a few milliseconds. UNVERIFIED values.
- Feed the switched rail through a divider to one GPIO as `rail_ok`. The stepper loop reads it every step (A.4).
- Relay K1, the one-shot, and the charge-pump arm signal from the base document are unchanged.
- Supply: 5 V at the node, sized for 0.5 A. Voltage drop over a 3 m drag-chain run matters at 5 V; either run 12 or 24 V in the chain with a buck converter on the node, or use heavy conductors. Torque falls roughly with coil voltage.

Node GPIO budget (C6): 12 stepper outputs (3 channels, third unpopulated), 2 TWAI, 1 permit in, 1 arm pulse out, 1 rail sense, 1 status LED = 18. Pin availability on your module is UNVERIFIED; avoid strapping pins GPIO4, 5, 8, 9, 15 for anything safety-related (base 7.4).

---

## A.8 CAN protocol field changes (base 5.4)

| Message | Change |
|---|---|
| MOVE_REL | `delta i32` is in full steps. `flags` bit0 = finish_plus (unchanged). |
| MOVE_DONE | `result`: 0 Done, 1 Stopped, 2 RailLost, 3 SoftLimit. `pos` in steps. |
| STATUS | `pos` in steps. `current_ma` is always 0. Flag bit2 means `suspect` (was stalled). Flag bit3 means `rail_ok` (was nfault). |
| CONFIG keys | 1 = step_period_us, 2 = overshoot_steps, 3 = soft_min, 4 = soft_max, 5 = dir_sign. |
| FAULT codes | Add 0x10 AxisNotResponding, 0x11 RailLostDuringMove. |

---

## A.9 Constants (replaces the motor block in base 9.2)

```rust
//! align_consts_stepper.rs: PLACEHOLDERS unless marked VERIFIED.
pub const STEPS_PER_REV: f32 = 2037.8864; // VERIFIED common variant; confirm per unit (A.6)
pub const SCREW_LEAD_UM: f32 = 500.0; // M6x0.5 per CAD model (A.12); confirm with 10-turn test
pub const LEVER_ARM_UM: [f32; 4] = [35_000.0; 4]; // per CAD model (A.12); mirror 1 mount UNVERIFIED
pub const DEFLECT_K: [f32; 4] = [2.0, 1.414, 2.0, 1.414]; // ASSUMED
pub const DIR_SIGN: [i32; 4] = [1; 4]; // UNVERIFIED, set at commissioning
pub const STEP_PERIOD_US: u64 = 5_000; // 200 pps, pending A.6
pub const ENERGIZE_DWELL_MS: u64 = 20;
pub const HOLD_BEFORE_RELEASE_MS: u64 = 100;
pub const SETTLE_MS: u64 = 200; // UNVERIFIED
pub const OVERSHOOT_STEPS: i32 = 120; // set to 2x measured backlash
pub const POKE_STEPS: i32 = 45; // CALCULATED for w = 4 mm, D = 1 m, L = 35 mm
pub const MAX_MOVE_STEPS: i32 = 4_000; // UNVERIFIED
pub const MAX_MOVE_MS: u64 = 25_000; // UNVERIFIED
pub const SOFT_LIMIT_MARGIN_FRAC: f32 = 0.10; // UNVERIFIED
pub const PREDICTION_MISMATCH_FRAC: f32 = 0.5; // UNVERIFIED
pub const PULL_IN_TORQUE_NM: f32 = 0.0343; // VERIFIED datasheet minimum
```

---

## A.10 Additions to the to-measure checklist (base 9.1)

1. Coil resistance and steps per revolution of each unit (A.6).
2. Torque margin test result and chosen step rate (A.6).
3. Backlash per axis in steps, measured with the beam.
4. Shaft offset and ear hole spacing of your units, for the bracket.
5. Voltage at the motor rail under two-phase load at the far end of the drag chain.
6. Whether mirror tilt shifts when coils release (shoot before and after release once per axis).

---

## A.12 Cloudray Series E mount geometry (from CAD)

Source: user-supplied STEP model "Cloudray Series E Second Mirror". The user confirmed both mounts are Series E. The model appears to be community-made, so every value is AS MODELED: confirm on the physical part. The primary (mirror 1) mount has not been checked against a model.

Frame: u = horizontal in the plate plane, z = vertical, n = screw axis. "Behind" means the knob side of the fixed plate.

| Item | Value | Status |
|---|---|---|
| Adjuster thread | M6 x 0.5 mm; pitch measured from modeled helices (0.496 to 0.504 mm per turn) | AS MODELED; confirm with 10-turn caliper test |
| Screw pattern | L with 35.0 mm arms; corner (pivot) screw at the bottom, on the side away from the beam tester | AS MODELED |
| Lever arm L | 35.0 mm for both end screws | AS MODELED |
| End screw spacing | 49.5 mm between the two motorized screws (diagonal) | CALCULATED |
| Horizontal-arm screw | Tilts about a vertical axis: steers horizontally, k = 2 | CALCULATED; verify by poking |
| Vertical-arm screw | Tilts about a horizontal axis: steers vertically, k = 1.414 | CALCULATED; verify by poking |
| Thumbscrew | 24.5 mm long overall, knob 10 mm diameter, knob end 14.9 mm behind the fixed plate, 3 mm tip | AS MODELED |
| Lock nut | 11 mm diameter, 3.5 mm thick, one per adjuster, against the back of the fixed plate | AS MODELED |
| Fixed plate | 45 mm wide, 8 mm thick | AS MODELED |
| Moving plate | 45 x 45 x 10 mm | AS MODELED |
| Mirror | 25 mm diameter, 3 mm thick, centered in the screw square (17.5 mm from each tilt axis) | AS MODELED |
| Springs | Two compression springs, about 6 mm diameter, on M4x25 screws that thread into the moving plate; located 24 mm from the corner screw along each arm | AS MODELED |
| Spring screw heads | 7 mm diameter, 16.0 mm behind the fixed plate | AS MODELED |
| Knob to spring screw | 11.0 mm center to center; 2.5 mm clear between knob and screw head | CALCULATED |
| Seats | 3 mm screw tip in a 2 mm hole at all three points: over-constrained, not cone-groove-flat | AS MODELED |

Design consequences:

- Coupler cup wall is limited to 1.5 to 2 mm by the adjacent spring screw head.
- The motor body must sit behind the spring screw heads: allow 16 mm plus clearance, then coupler, then the 20 mm motor body. Budget 45 to 50 mm behind the fixed plate.
- Two 28 mm motor bodies fit at 49.5 mm spacing without splaying.
- Lock nuts must be removed or left loose on motorized screws. This raises the priority of the vibration creep test.
- Three hole seats predict stick-slip. Run the hysteresis characterization (base 3.4) early; it is the deciding data for machining custom kinematic mounts.

---

## A.11 Sources

- Kiatronics 28BYJ-48 5V datasheet: https://pdf.direnc.net/upload/28byj-48-reduktorlu-step-motor-datasheet.pdf
- Alternate 28BYJ-48 datasheet (pull-in torque, rates): https://www.sgbotic.com/products/datasheets/robotics/28BYJ48.pdf
- Same datasheet figures (friction torque): https://cdn.shopify.com/s/files/1/0015/7571/4865/files/datasheet_28BYJ-48.pdf
- Gear tooth count and 2038 steps: https://pcbsync.com/28byj-48-uln2003-arduino/
- Measured 2037.8864 steps and 27 ohm coil: https://github.com/scottbez1/splitflap/wiki/Motor-info
- Variant buying guide (16:1, 64:1, 63.68:1): https://paper.dropbox.com/doc/Lhd8hrS392gznzeDE78pV
- Vendor torque, coil resistance, shaft: https://www.robotshop.com/products/28byj-48-64-641-stepper-gearmotor
- Dimensions: https://www.solarbotics.com/product/22310/
- Wire colors, mass, current: https://canadarobotix.com/products/1926
- Half-step preference note: https://www.crcibernetica.com/28byj-48-stepper-motor-with-uln2003-driver
- ULN2003A facts: not rechecked here; confirm against the TI datasheet.
