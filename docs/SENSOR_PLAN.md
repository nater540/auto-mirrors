# Sensor node firmware: plan

Status: PLAN, revision 3, 2026-11-15. Nothing here is built.
Revision 3 answers an adversarial review of revision 2; section 15 maps each finding to its resolution.
The owner decided that the sensor firmware belongs in this repository, written here; that the CAN protocol may change
as needed; and that five push buttons stand in for the detector until it exists.
Precedence is unchanged: the hand-off, the addendum, and the reference win where they disagree, except where this plan
proposes a change to one of them and says so (section 13).

## 0. Problems first

1. **The method depends on a detector nobody has chosen.** Energy integration (reference 2.4) works for a
   DC-coupled thermal detector such as a thermopile. A pyroelectric quad is AC-coupled and integrates to about zero
   over a pulse; a fast photon detector gives a pulse as short as the dot. This plan ASSUMES a DC-coupled thermal
   detector, puts the method behind its own trait (section 5), and needs the owner's sign-off on the assumption.
2. **Shots of one job can overlap in the detector.** The canned job fires A, B, and C with only gantry travel
   between them (reference 6.7). If travel is shorter than about five time constants, shot B lands on the decay of
   shot A. Section 6.4 handles it by superposition, ending each shot at the next and adding its tail analytically.
3. **The sensor cannot see the laser fire.** The fire input is on the coordinator. A SHOT message carries it, and
   the sample buffer makes its latency harmless (section 3.1).
4. **One development board cannot hold both the motor node and five buttons.** The motor node uses 12 pins and
   reserves 7 more; strapping pins are risky for buttons. A second ESP32-C6 development board is the recommendation
   (section 7.3).
5. **The coordinator side changes too.** It needs per-shot result codes, a timeout, a way to ask for a lost reading,
   and a state to wait for readings in. Those land in `align-core` and in reference 7.3 (section 13).
6. **Push buttons are not a beam.** They exercise timing, protocol, checks, and fault paths through a simulated
   thermal response; the numbers that come out are the simulator's, not a detector's.

## 1. Requirements and where they come from

| # | Requirement | Source |
|---|---|---|
| R1 | Report normalized X, Y, and the sum for each shot, over CAN, as node 3 | Hand-off 1, 3.1 |
| R2 | `X = ((A + D) - (B + C)) / S`, `Y = ((A + B) - (C + D)) / S`, on baseline-subtracted integrals | Reference 2.1 |
| R3 | Integrate each quadrant minus its extrapolated baseline over the decay; stop at 5 tau or back at baseline | Reference 2.4 |
| R4 | Reject saturated, negative-quadrant, low-sum, edge, unrepeatable, and off-energy readings | Reference 2.3 |
| R5 | Nothing may depend on fast feedback; a reading may take seconds | Hand-off 2, item 5 |
| R6 | Keep the calibration (`tau_cal`, `S_ref`) across reboots | Reference 3.6 |
| R7 | The sensor never moves or fires anything | Hand-off 3.3, 7 |
| R8 | `no_std` libraries, no panics, typed errors, tagged config, tests first | CLAUDE.md |
| R9 | Five buttons (four corners and the center) drive a simulated beam until the detector exists | Owner, 2026-11-15 |
| R10 | Commissioning must work on the machine, where the sensor rides the gantry with only five wires | Hand-off 3.1 |

## 2. System context

```text
 coordinator                          CAN                          sensor node (node 3)
 FootSW pulse; Ruida runs the job
 fire burst at A  --- SHOT seq, point=A, edge_age --->              places the shot in its sample buffer
 fire burst at B  --- SHOT seq+1, point=B --------->                same; A's window ends where B fires
 fire burst at C  --- SHOT seq+2, point=C --------->                same
 Status low: job done; waits in CollectingReadings                  each shot finishes when its decay ends
                  <--- SENSOR seq, result, X, Y, sum ---            (in shot order)
 missing one?     --- SENSOR_RESEND seq ----------->
                  <--- SENSOR seq ... ---------------               from the last eight results
 three results: Shots{a, b, c} -> Aligner, or an invalid measurement
```

## 3. Protocol changes

Classes 0xA to 0xD are new. Every node decodes them; nodes that do not use them ignore them without logging.

### 3.1 SHOT (class 0xA, coordinator to all, DLC 4)

| Byte | Field | Meaning |
|---|---|---|
| 0 | `seq u8` | Shot number; the SENSOR reply echoes it |
| 1 | `point u8` | 0 A, 1 B, 2 C, 0xFF not a measurement point; logged by the sensor |
| 2..3 | `edge_age_10us u16` LE | Time from the start of the fire burst to queuing the frame, in 10 us units (655 ms range). 0xFFFF: unknown |

- **One SHOT per burst, not per edge.** With an RF tube (LPWM) or PWM inside a dot, one dot is many edges. The
  coordinator groups edges separated by less than `burst_gap_ms` into one burst. Whether a glass tube in Ruida dot
  mode gives one L-ON edge per dot is UNVERIFIED: a scope check on the Mod5 settles it (section 14).
- **Placement:** the sensor timestamps the frame on reception and places the fire at
  `rx_time - edge_age - frame_time`. A 4-byte standard frame is at most 98 bits with worst-case stuffing, 0.39 ms at
  250 kbit/s (CALCULATED). Unknown age: the fire is searched for in the buffer before reception (section 6.2).
- **Priority:** 0xA loses arbitration to every older class. A late SHOT costs nothing as long as it arrives before
  its samples leave the buffer, which section 6.1 sizes for.
- **Duplicates:** the sensor ignores a SHOT whose `seq` it has seen within `dedup_window_ms`, and forgets every seq
  when the coordinator's heartbeat uptime goes backwards (a coordinator reboot).

### 3.2 SENSOR (class 0x7, sensor to coordinator, DLC 8), redefined

| Byte | Field | Meaning |
|---|---|---|
| 0 | `seq u8` | The SHOT `seq` it answers; in bench mode, the sensor's own counter |
| 1 | `status u8` | Bits 0 to 4: result code. Bit 5: calibrated. Bit 6: reserved, ignored on decode. Bit 7: bench shot |
| 2..3 | `h_q15 i16` LE | X in Q15 (32767 is about +1.0), saturating; 0 when the result has no position |
| 4..5 | `v_q15 i16` LE | Y in Q15 |
| 6..7 | `sum_q u16` LE | `S / S_ref`, 32768 is 1.0. 65535 means "at or above 2.0", never a usable sum |

`sum_q` is filled for every result that integrated anything, NoShot included, because coarse acquisition and the
"check the sum before each further step" rule (reference 3.5, 7.5) need it. It is 0 only for TimedOut and
FrontEndFault.

Result codes, in priority order (the first that applies is sent):

| Code | Name | Position | Meaning |
|---|---|---|---|
| 9 | FrontEndFault | no | The front end failed during the shot |
| 7 | TimedOut | no | The decay did not end within the cap, or its samples left the buffer |
| 4 | BaselineUnstable | no | The pre-burst baseline fit failed its flatness test |
| 1 | NoShot | no | No rise at the fire: no fire, beam off the sensor, or clipped upstream |
| 2 | Saturated | not trusted | A raw sample reached the saturation level during the shot |
| 6 | Overlap | not trusted | The previous shot's tail model left a residual above its test |
| 3 | NegativeQuadrant | not trusted | A quadrant integral below `-3` of its own propagated sigma |
| 0 | Valid | yes | Every node-side check passed |

Codes 5 and 8 of revision 2 are gone: the three-sample fit is deferred (section 6.5), and calibration is now a flag.
The calibrated flag clears when `S_ref` or `tau_cal` is a default; `sum_q` is then relative to the default `S_ref`.

### 3.3 SENSOR_RESEND (class 0xB, coordinator to sensor, DLC 1)

`seq u8`. The sensor resends that result if it is among its last eight, and otherwise sends nothing. A lost SENSOR
then costs one frame, not a three-shot job and three laser fires.

### 3.4 SENSOR_CMD (class 0xC, coordinator to sensor, DLC 4) and SENSOR_CMD_REPLY (class 0xD, DLC 8)

Commissioning from the coordinator console, since the drag chain has no USB (R10).

| `op u8` | Argument (`arg u16`, byte 2..3) | Does |
|---|---|---|
| 1 Noise | seconds | Measures sigma per quadrant, beam off |
| 2 Tau | none | Fits tau per quadrant on the next shot's decay |
| 3 SRef | shots | Averages the sum of the next n shots, beam on the sensor (not necessarily centered) |
| 4 Save | none | Writes the calibration record |
| 5 Report | quadrant index | Replies with that quadrant's tau and sigma |
| 6 Bench | 0 or 1 | Leaves or enters bench mode |

Reply: `op u8, status u8 (0 ok, 1 busy, 2 refused, 3 failed), value_a i32` and two spare bytes, one reply per
command; Report replies carry tau in microseconds and, in a second reply, sigma in counts. The USB console offers the
same commands for the bench.

### 3.5 Existing messages

| Message | Change |
|---|---|
| HEARTBEAT (0x8) | DLC 7: adds `proto_version u8`. A node that sees a different version from the coordinator refuses moves (motor) or shots (sensor) and reports FAULT 0x34 ProtocolMismatch. Sensor `state`: 0 NotReady, 1 Idle, 2 Busy, 3 Bench, 4 FrontEndFault |
| FAULT (0x2) | New: 0x30 FrontEndFault, 0x31 BufferOverrun, 0x32 CalibrationMissing, 0x33 UnexpectedFire, 0x34 ProtocolMismatch. Persistence failures reuse 0x2D |
| ESTOP, INHIBIT | Ignored by the sensor: it drives nothing |

FAULT 0x33 UnexpectedFire: a rise with no SHOT within `unexpected_fire_window_ms`, outside bench mode. Evidence for
the coordinator's log, never relied on: the fire input and the relay are the interlock (reference 7.1).

### 3.6 Code that must change with the protocol

`frame.rs` (the "largest class is 0x9" comment, `compose_id`'s fallback reasoning), the tests that assert 0xA to 0xF
are undefined (`frame.rs`, `message.rs`), the motor node's decode-warning path (it must ignore sensor classes
quietly), and both firmware images, which must be rebuilt together because of `proto_version`.

## 4. Hardware assumptions

| Item | Assumption | Tag |
|---|---|---|
| MCU | ESP32-C6, as every node | FIRM (hand-off 3.1) |
| Detector | DC-coupled thermal quad (thermopile type): a linear first-order response, decaying back to baseline | ASSUMED, for sign-off |
| Front end | Four channels sampled together at a fixed rate, as signed counts | UNVERIFIED |
| Floating point | None in hardware (riscv32imac): every `f32` operation is software | VERIFIED (target) |
| Dot length | 5 to 50 ms (reference 6.7) | UNVERIFIED |
| Time constant | "tens of ms to seconds" (reference 2.4); measured per quadrant at commissioning | UNVERIFIED |
| Quadrant orientation | A upper right, B upper left, C lower left, D lower right, seen along the beam toward the detector | ASSUMED; the measured Jacobian absorbs any sign |

## 5. Structure

| Piece | Kind | Contents |
|---|---|---|
| `crates/sensor-logic` | `no_std`, host-tested | Sample ring, shot queue, baseline, tail model, the integration method, checks, the sensor state machine behind a `SensorIo` trait, console and SENSOR_CMD handling, calibration record |
| `sensor-logic::sim` | `no_std`, host-tested | The simulated thermal beam and the button logic (section 7). Its numbers are scenario parameters, not configuration |
| `firmware/sensor` | binary `align-sensor` | Front end, buttons, TWAI, flash, USB console. Pins in `board.rs`. `FLASH_NOTICE.txt` |
| `QuadrantFrontEnd` | trait | `async fn sample(&mut self) -> Result<[i32; 4], FrontEndFault>`, once per sample period |
| `ShotMethod` | trait | Turns a burst of placed shots and their samples into per-quadrant energies and a quality. `ThermalIntegration` is the first; a pyroelectric or photon detector gets its own |

Reused from `align-core`: `Quadrants::normalize` only. `Quadrants::check` compares values against a per-sample full
scale and one sigma, which fits neither integrals nor per-quadrant noise; the sensor has its own checks (6.6).

## 6. Processing (`ThermalIntegration`)

### 6.1 Sample ring

Entries of `[i32; 4]` with a `u32` sample index; index arithmetic is wrapping, and a test covers the wrap. Size in
samples: `rate * (baseline_window + max_burst + cap + resend_hold)`, held to a RAM budget of 128 KiB (8,192 entries).
The rate follows the calibration: at least 10 samples per `tau_min` (reference 2.4), and the cap is
`end_tau_multiple * tau_max`. If the budget cannot hold both, the console says which calibration is unworkable
instead of silently truncating. A shot whose window left the ring is TimedOut, with FAULT 0x31.

### 6.2 Placing a shot

1. The fire index comes from SHOT (3.1). With an unknown age, it is the first rise in the 200 ms before reception.
2. A rise test confirms a fire: within `rise_search_ms` after the fire index, the residual sum (samples minus the
   burst model so far, 6.3 and 6.4) exceeds `rise_sigma` of its propagated sigma. No rise: NoShot.
3. Integration windows are anchored to the fire index, not to the rise crossing: each starts `pre_fire_ms` before
   it, so the leading edge and the guard samples are inside.

### 6.3 Baseline: once per burst

A burst is a run of shots each placed before the previous one ended. The baseline is fitted once, before the first
shot of a burst: mean and slope per quadrant, by closed-form least squares over `baseline_window_ms` (reference 2.4,
0.5 s) ending `pre_fire_ms` before the first fire. Residual sigma above `baseline_flat_sigma` times the channel
noise: BaselineUnstable for every shot of the burst. Later shots use the same line: it is anchored before any of them,
so no shot's decay is in it.

### 6.4 Integration and overlap, by superposition

The response is linear and, after the dot ends, a pure exponential decay with the quadrant's own `tau_q`. For each
shot k of a burst, per quadrant:

1. Subtract the baseline line and the tail models of shots 1 to k-1 from the samples.
2. Integrate from `pre_fire_ms` before fire k until the first of: the next shot's fire index minus `pre_fire_ms`
   (a cut); the residual back within 2 sigma for `end_hold_samples`; `end_tau_multiple * tau_max` (TimedOut).
3. Fit shot k's tail: amplitude `a_q` of `exp(-(t - t_k) / tau_q)` over the decay after `dot_max_ms`, with `tau_q`
   fixed from calibration, by one-parameter linear least squares. If shot k was cut, add the analytic remainder
   `a_q * tau_q * exp(-(t_cut - t_k) / tau_q)` to its integral. That is shot k's tail model for the shots after it.
4. The tail fit's residual, against the noise, decides Overlap.

Shots finish in order: shot k is final only when its own end condition holds or the next shot has been placed and
cut it, so a late SHOT cannot finalize a shot that the next fire would have corrected. Up to four shots wait in a
queue. Every exponential comes from a per-sample recurrence (one constant factor `exp(-dt / tau_q)` per quadrant), so
a shot costs a few multiplications per sample and no `expf` inside a loop.

### 6.5 Three-sample fit: deferred

Reference 2.4's `y_inf` is the asymptote of a step response; after a dot the detector returns to baseline, so for this
use `y_inf` is the baseline, not the signal. Which fitted quantity would stand for the energy, and under what dot
length, is open until a detector exists. Not built in S1.

### 6.6 Checks

- **Saturation** on the raw samples: any sample of the shot's window at or above `saturation_frac` of full scale.
- **Negative quadrant** against each quadrant's propagated sigma: the variance of an integral of N samples is
  `N sigma_q^2` plus the variance of the integrated baseline line, `sigma_q^2 * u^T (M^T M)^-1 u`, where `M` is the
  baseline fit's design matrix and `u = [N, sum of t]` over the window (CALCULATED, ordinary least squares).
- **Low sum, edge, repeat, energy:** at the coordinator, which has `S_ref` relative sums and every shot
  (`Reading::check`, `check_repeat`, `check_energy`).
- Then `Quadrants::normalize`, Q15 with saturation, and `sum_q`.

### 6.7 Arithmetic and time

Samples are `i32`; integrals `i64` (8,192 samples of at most `2^31` sum to at most `2^44`, far inside `2^63`).
Fits, tails, and normalization are `f32` with `libm` in software.
Processing runs in its own task (section 9), never in the sampler's tick; its time per shot is measured on the board,
as an S3 acceptance item. Every division is checked: a zero or non-finite value becomes a result code.

### 6.8 Configuration (`align_core::config::SensorTiming`)

All UNVERIFIED unless marked; most follow the calibration rather than being fixed.

| Value | Start | Tag |
|---|---|---|
| `baseline_window_ms` | 500 | FIRM (reference 2.4) |
| `pre_fire_ms` | 20 | UNVERIFIED |
| `baseline_flat_sigma` | 3.0 | UNVERIFIED |
| `rise_search_ms` | 100 | UNVERIFIED |
| `rise_sigma` | 10.0 | UNVERIFIED |
| `end_tau_multiple` | 5.0 | FIRM (reference 2.4) |
| `end_hold_samples` | 3 | UNVERIFIED |
| `dot_max_ms` | 50 | UNVERIFIED (reference 6.7 upper end) |
| `tau_default_ms` | 100 | UNVERIFIED, until commissioning |
| `samples_per_tau_min` | 10 | FIRM (reference 2.4) |
| `ring_budget_bytes` | 131,072 | UNVERIFIED |
| `dedup_window_ms` | 2,000 | UNVERIFIED |
| `unexpected_fire_window_ms` | 200 | UNVERIFIED |
| `burst_gap_ms` (coordinator) | 5 | UNVERIFIED, until the Mod5 scope check |

## 7. Simulated beam and the five buttons

### 7.1 Modes

- **Bench mode** (console `bench on`, or SENSOR_CMD Bench): a button press fires a simulated shot and the sensor
  reports it with the bench bit set and its own `seq`. No SHOT is needed.
- **Bus mode** (default): buttons only set where the beam is; a SHOT fires the simulated shot at the held position
  (nothing held: centered). The sensor never reports a shot without a SHOT, so there are no double replies.

### 7.2 Buttons

Momentary, normally open, to ground, internal pull-ups, active low. Debounce 20 ms; a chord is every button pressed
within 50 ms of the first. Positions, as a fraction `corner_offset` of the beam radius:

| Pressed | Beam at |
|---|---|
| One corner | That corner: `(+-c, +-c)` |
| Two adjacent corners | The edge midpoint between them: `(+-c, 0)` or `(0, +-c)` |
| Center | `(0, 0)` |
| Center and one corner | Halfway to that corner: `(+-c/2, +-c/2)` |
| Opposite corners, or three or more | Rejected: logged, no shot |

The debounce, chord, and position logic is a pure function of sampled levels and time, host-tested.

### 7.3 Pins

The motor node's board uses GPIO 0, 2, 3, 6, 7, 10, 11, 18 to 22 and reserves 23 (TWAI receive), 12 and 13 (USB),
and 16 and 17 (UART0). Strapping pins 4, 5, 8, 9, and 15 can change the boot if a button is held at reset (GPIO 9 low
enters download mode). **Recommendation: a second ESP32-C6 development board for the sensor**, with the buttons on
GPIO 18 to 21 and 23 and TWAI on GPIO 22 as on the motor node. Two boards also allow a real two-node bus: both
controllers in no-transceiver mode with their TWAI pins wired together, open drain, which is what esp-hal's
`new_no_transceiver` is for. On one board, the fallback is GPIO 1, 16, 17, 23, and 4, with UART0 and the TWAI receive
reservation given up and a warning not to hold the GPIO 4 button at reset; that board then cannot run the motor node
and the sensor at once.

### 7.4 Thermal model

- Beam: Gaussian of radius `w` at `(x0, y0)`; quadrant fractions of an ideal gapless quad from
  `right = (1 + erf(sqrt(2) x0 / w)) / 2` and `top` likewise (reference 2.2). `corner_offset` 0.3 gives X about 0.45
  (CALCULATED: `erf(0.424) = 0.451`), inside the solver range of 0.6.
- Response per quadrant: a dot of length `dot_ms` through a first-order system of time constant `tau_q`, so the
  integral is `gain_q * E * fraction_q` whatever `tau_q` is, plus a baseline offset and slope, Gaussian noise, and
  clipping at full scale.
- Parameters are scenario values in `sensor-logic::sim`, settable from the console: `corner_offset`, energy, dot
  length, `tau_q` per quadrant, noise, drift, gain per quadrant, full scale.
- Random numbers: the ESP32-C6 hardware RNG on the board, a seeded generator on the host, behind one trait.
- Fault injection: `sim energy 0` (NoShot), `sim energy 20` (Saturated), `sim drift 50` (BaselineUnstable),
  `sim fault` (FrontEndFault), `shot [age]` (a synthetic SHOT through the same receive path, for testing placement
  without a coordinator), `sim burst 300` (two shots 300 ms apart, both with SHOT: both Valid when the tail model
  works, Overlap when `tau` is set wrong on purpose).
- Gaps: no detector gap, no non-Gaussian mode, no crosstalk. It tests the firmware, not the optics.

## 8. Calibration record

Format version; per quadrant `tau_us` and `sigma_counts`; `S_ref` and the power it was taken at; written with
sequential-storage on a dedicated `sensor` data partition, never the motor node's `nvs`. A custom partition table in
`firmware/sensor` provides it; a board flashed with another image has no such partition, and the sensor then runs
uncalibrated rather than touching another image's data. (The motor node should move to its own partition label too,
so that its erase-on-unreadable rule can never reach another image's data; that is a separate change.)

How `S_ref` relates to laser power (coarse acquisition runs at a different power; `Reading::check` takes "the
reference sum for the power in use") is open: section 14.

## 9. Firmware tasks

| Task | Priority | Does |
|---|---|---|
| `sampler` | high | Reads the front end on a `Ticker`; appends to the ring; nothing else. A missed tick raises FAULT 0x31 |
| `processor` | low | Places, integrates, checks, and reports shots from the ring, in order |
| `can` | normal | SHOT, SENSOR_RESEND, SENSOR_CMD in; SENSOR, replies, HEARTBEAT, FAULT out; bus-off recovery |
| `buttons` | normal | Polls the five inputs every 5 ms into the debounce logic |
| `console` | low | The USB console |
| `persist` | low | Reads the calibration at boot; writes it on Save, never while a shot is in progress |

The ring is single-writer (the sampler) with an index the processor reads; samples older than the processor's
oldest pending shot may be overwritten, and the sampler reports when it would overwrite a pending one.

On a coordinator reboot (its uptime goes backwards in the heartbeat) or heartbeat loss over `1 s`, the sensor drops
pending shots and forgets seen sequence numbers.

## 10. Failure modes

| Failure | Detected by | Outcome |
|---|---|---|
| No fire, or beam off the sensor | No rise at the fire index | NoShot, with the measured sum |
| SHOT lost | Coordinator timeout | Invalid shot |
| SENSOR lost | Coordinator timeout | SENSOR_RESEND; then an invalid shot |
| Duplicate SHOT | Same seq within the dedup window | Ignored |
| Coordinator reboot mid-job | Uptime backwards | Pending shots dropped, sequence numbers forgotten |
| Sensor reboot mid-job | Coordinator sees uptime backwards | Measurement aborted at the coordinator |
| Overlapping shots | Fire inside the previous window | Cut and tail-corrected; bad tail fit: Overlap |
| Drift | Baseline residual | BaselineUnstable |
| Saturation | Raw sample at full scale | Saturated |
| Front end stops | Fault or sample timeout | FrontEndFault, heartbeat state 4 |
| Sampler late | Missed tick | FAULT 0x31; affected shots TimedOut |
| Uncalibrated | Default values | Calibrated bit clear; coordinator policy (section 13) |
| Protocol mismatch | Heartbeat `proto_version` | FAULT 0x34; shots refused |
| Fire with no SHOT | Rise outside bench mode | FAULT 0x33, logged only |

## 11. Testing

- **Units:** reference 2.1 and 2.2 against worked numbers; the integral of a dot through a first-order system;
  a linear baseline removed exactly; the closed-form line fit; the propagated sigma against a Monte Carlo estimate;
  the exponential recurrence against `expf`.
- **Properties:** X and Y do not depend on energy or a common gain; baseline offset and slope do not move them; any
  sample stream and any frame sequence yields results without a panic; two and three overlapping shots recover each
  shot's position within noise for any spacing above `dot_max_ms`; a shot never rising is NoShot.
- **Edge cases:** sample index wrap; a shot spanning the ring's wrap; a SHOT arriving after its samples are
  processed; a SHOT arriving before the rise; a coordinator reboot between shots; four shots queued.
- **Protocol:** round trips for the new classes; motor nodes ignore them; strict decoding of SHOT and SENSOR_CMD,
  lenient on SENSOR's reserved bit.
- **Simulator:** `align-sim` runs the thermal model through `sensor-logic`, so the closed-loop tests run the full
  signal path, overlap included.
- **Board (S3):** each button position gives the expected signs and size; chords give midpoints; every fault
  injection gives its code; `shot` exercises placement; processing time per shot measured; with two boards, SHOT and
  SENSOR cross a real two-node bus.

## 12. Milestones

They do not block M5: the coordinator can be built against the simulated sensor.

| Milestone | Contents | Accepted when | Needs from the owner |
|---|---|---|---|
| S0 | Protocol (section 3), the motor node updated to ignore the new classes, `proto_version` | Round trips, boundaries, mutation testing clean; both images rebuilt | Sign-off on section 3 |
| S1 | `sensor-logic`: ring, queue, placement, baseline, superposition, checks, state machine, SENSOR_CMD | Section 11 units, properties, and edge cases | Sign-off on the detector assumption |
| S2 | `sensor-logic::sim` and the buttons; the `align-sim` sensor scenario | Closed-loop tests pass through the full signal path | Nothing |
| S3 | `firmware/sensor`: board, partition table, FLASH_NOTICE, justfile recipes, workspace entries | Section 11 board checks | A second board (recommended) and five buttons |
| S4 | The real front end and, if the detector is not thermal, its `ShotMethod` | Swaps in below the two traits | Open item 6 |
| S5 | Commissioning on the machine through the coordinator console | Calibration saved; canned-job spacing checked | The sensor mounted; the Mod5 scope check |

## 13. Changes this plan proposes elsewhere (owner sign-off)

- **Reference 7.3:** Measuring exits on Status low into a new state, CollectingReadings, which waits for every
  SENSOR (with SENSOR_RESEND) up to `reading_timeout_ms = baseline_window + cap + 1 s` after the last SHOT, then goes
  to Moving or counts an invalid measurement.
- **`align-core`:** `Reading` gains the sensor's result code and calibrated flag; `SensorFault` gains `TimedOut`;
  the `Aligner`'s invalid-shot counting treats every non-Valid result as invalid. Uncalibrated readings: accepted in
  the simulator and on the bench; on the machine the coordinator refuses to align until the sensor is calibrated.
- **CLAUDE.md:** the new crates, the protocol classes, and these decisions, once S0 lands.

## 14. Questions for the owner

1. Sign-off on section 3 and section 13.
2. Sign-off on the DC-coupled thermal detector assumption, or anything known that contradicts it.
3. A second ESP32-C6 development board for the sensor (recommended), or the one-board fallback.
4. How should `S_ref` scale with laser power? Proposal: store it with its power, and let the coordinator scale it
   linearly above the lasing threshold (UNVERIFIED until measured).
5. On the Mod5, with a scope: how many edges does one Ruida dot produce on the fire line?

## 15. Review disposition (revision 2 review, 17 findings)

| # | Finding | Resolution |
|---|---|---|
| 1 | Overlap: shot A swallows shot B; ordering and circularity | 6.3 and 6.4: baseline once per burst, cut at the next fire, analytic remainder, in-order finalization, queue of four |
| 2 | Uncalibrated counted invalid; one code cannot carry two facts | 3.2: calibrated flag bit; priority order; policy in 13 |
| 3 | Three-sample fit meaningless for a dot | 6.5: deferred |
| 4 | `Quadrants::check` does not fit integrals or per-quadrant sigma | 5, 6.6: own checks, propagated sigma; reuse only `normalize` |
| 5 | Method assumes a thermopile | 0, 4, 5: ASSUMED for sign-off; `ShotMethod` trait |
| 6 | Pins and flash conflict on one board | 7.3: second board; 8: own partition |
| 7 | Board cannot exercise SHOT | 7.4 `shot` command; two-board bus; host SHOT tests |
| 8 | Processing inside the sampler tick, no FPU | 6.7, 9: separate task, recurrence, measured |
| 9 | Rate, buffer, and cap inconsistent; "3 s" misread | 6.1, 6.8: derived from calibration, budgeted, tagged |
| 10 | Coordinator cannot use results; no timeout; lost reading costs a job | 3.2, 3.3, 13 |
| 11 | Dedup across reboots; bench mode undefined; double replies | 3.1, 7.1, 9 |
| 12 | Commissioning only over USB at the head | 3.4: SENSOR_CMD |
| 13 | Orientation attributed to the reference | 4: ASSUMED |
| 14 | Per-quadrant tau inconsistent; model couples tau and gain | 6.4 per-quadrant tau; 7.4 normalized response |
| 15 | Rise and baseline circular; leading edge lost | 6.2: anchored to the fire index |
| 16 | Protocol details (age range, edges, point, sum, codes, bits, code to change) | 3.1 to 3.6 |
| 17 | Simulator config in `align_core`; wrong expectation for a double pulse | 5, 7.4 |
