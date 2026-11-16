# Sensor node firmware: plan

Status: DRAFT for the owner, 2026-11-15. Nothing here is built. Precedence is unchanged: the hand-off, the addendum,
and the reference win over this plan where they disagree.

## 0. Problems first

1. **The hand-off puts the sensor out of scope** ("Sensor hardware is designed separately", section 1; "do not expand
   scope into sensor hardware design", section 7). This plan covers the firmware only. It treats the detector, its
   amplifiers, and its ADC as a black box behind one trait, and asks the owner for everything about them.
2. **The sensor node cannot see the laser fire.** Reference 2.4 integrates "from the fire-line timestamp", but the
   fire line is wired to the coordinator (reference 6.6), not to the head. Section 3 proposes a new message for it.
3. **The SENSOR message is too thin for the checks reference 2.3 requires.** It carries X, Y, and the sum, not the
   four quadrants. The saturation and negative-quadrant checks therefore run on the sensor node, and the node needs
   a way to say a reading failed them. Section 3 proposes meanings for the `kind` byte.
4. **The sum has no defined scale** (`sum u16`, reference 5.4). Section 3 proposes one.
5. **Nothing is known about the detector** (hand-off open item 6): type, time constant, noise, gain, ADC, and the
   beam radius at the head. Every number below that depends on them is a placeholder, tagged UNVERIFIED.

## 1. What the sensor node does

It sits at the laser head entrance, on the CAN bus as node 3 (`align_proto::NodeId::SENSOR`). For every shot the
coordinator fires at the gantry positions A, B, and C, it reports one SENSOR message: the beam position (X, Y) and the
sum, normalized and checked. It never fires the laser and never moves anything. The coordinator pairs the three
readings into `align_core::measurement::Shots`, as its `BeamSensor` implementation (M5).

Between shots it samples continuously, so that a baseline exists before every shot and a late trigger loses no data.

## 2. Structure

Same pattern as the motor node: decisions in a host-tested library, hardware in a thin firmware crate.

| Piece | Kind | Contents |
|---|---|---|
| `crates/sensor-logic` | `no_std`, host-tested | Sample ring buffer, baseline fit, shot integration, end-of-shot detection, the three-sample exponential fit, validity checks, Q15 conversion, the sensor state machine behind a `SensorIo` trait, its console commands, its flash record |
| `firmware/sensor` | ESP32-C6 binary `align-sensor` | Wires `sensor-logic` to the front end, TWAI, flash, and the USB console. Pins in `board.rs` |
| `QuadrantFrontEnd` trait | in `sensor-logic` | `async fn sample(&mut self) -> Result<[i32; 4], FrontEndFault>`, one call per sample period. The real detector board implements it; so does a simulated front end |
| Simulated front end | in `sensor-logic` and `align-sim` | Thermal responses `y(t) = y_inf + (y0 - y_inf) exp(-t / tau)` with noise, baseline drift, and per-quadrant tau, from a seed. Lets the firmware run on the development board with no detector |

Reuse, not rewrite: `align_core::quadrant::Quadrants::normalize` (reference 2.1) and `Quadrants::check`
(saturation, negative quadrant) run on the node; `Reading::check`, `check_repeat`, and `check_energy` need more than
one shot or the reference sum, and stay at the coordinator.

## 3. Protocol changes (need the owner's sign-off)

| Change | Proposal | Why |
|---|---|---|
| New class 0xA, SHOT | coordinator to all, DLC 5: `seq u8, fire_us u32` (coordinator time of the fire edge) | Tells the sensor a shot happened and which one. The sensor answers with a SENSOR carrying the same `seq` |
| SENSOR `seq` | echoes the SHOT `seq` | Pairs each reading with its shot (A, B, C) without guessing from order |
| SENSOR `kind` | 0 valid; 1 no shot found (sum low); 2 saturated; 3 negative quadrant (baseline error); 4 decay fit rejected; 5 baseline unstable; 6 timed out | The coordinator cannot check quadrant data it never receives |
| SENSOR `sum` | `S / S_ref`, scaled so that 32768 is 1.0 and saturating at 65535 (about 2.0) | The coordinator's checks are all relative to `S_ref` (reference 2.3); the absolute units of the front end never leave the node |

Class 0xA is the first free class; a higher class number means lower priority, which suits a message that is late by
a frame without harm (below).

**Why a CAN trigger is good enough:** the node keeps a ring buffer, so it integrates from the samples around the
rise, not from the moment the SHOT arrives. A frame at 250 kbit/s takes under a millisecond; thermal time constants
are tens of milliseconds to seconds (reference 2.4, UNVERIFIED). The SHOT only has to arrive before the buffer
overwrites the shot, which a buffer of several seconds guarantees.

**Fallback:** the node can also find a shot by itself (a rise of the sum above the baseline by a threshold). That is
for the bench, where no coordinator fires the laser. On the machine, a reading without a SHOT is reported but marked.

## 4. Shot processing (reference 2.4, energy integration)

1. **Baseline:** fit mean and slope to each quadrant over the 0.5 s before the rise (reference 2.4, step 1).
2. **Integrate:** from the rise, integrate each quadrant minus its extrapolated baseline (step 2).
3. **End:** stop at `5 * tau_cal`, or when the sum is back within `2 sigma` of the baseline (step 3).
4. **Check:** `Quadrants::check` for saturation and negative quadrants, then `Quadrants::normalize` for X, Y, S
   (step 4).
5. **Report:** Q15 for X and Y, `S / S_ref` for the sum, `kind` from the first failed check.

The three-sample exponential fit (reference 2.4) is the alternative for long time constants, where waiting for
`5 * tau` takes too long: `y_inf = (y1*y3 - y2^2) / (y1 + y3 - 2*y2)`, rejected when the denominator is under about
`5 sigma` or the fitted tau is outside `[0.3, 3] * tau_cal`. Which method the machine uses is decided by the measured
tau (open item 6); both are built and tested.

Tagged values, all in `align_core::config` when built:

| Value | Starting point | Tag |
|---|---|---|
| Baseline window | 0.5 s | FIRM (reference 2.4) |
| Samples per tau | at least 10 | FIRM (reference 2.4) |
| Integration end | 5 tau, or the sum within 2 sigma of the baseline | FIRM (reference 2.4) |
| Fit rejection | denominator under 5 sigma; tau outside [0.3, 3] tau_cal | UNVERIFIED (reference 2.4) |
| Sample rate | set from tau_cal; 100 Hz as a placeholder | UNVERIFIED |
| Ring buffer | 20 s of samples (4 channels x 4 bytes x 100 Hz x 20 s = 32,000 bytes) | UNVERIFIED |
| Self-trigger threshold | sum rise above 10 sigma of the baseline | UNVERIFIED |

## 5. Commissioning and persistence

Reference 3.6 lists a Sensor record, `tau_cal, S_ref`, written at commissioning. The node keeps it in flash the way
the motor node keeps positions (sequential-storage on the `nvs` partition), with a format version. The bench console
measures them:

- `raw`: stream the four quadrant samples, for checking the front end and its noise.
- `tau`: fire, then fit tau per quadrant from the decay, and store the median as `tau_cal`.
- `sref`: average the sum of N shots on the centered beam and store it as `S_ref`.
- `shot`: wait for a self-triggered shot and print the reading and every check.

## 6. Testing

- **Unit:** the reference 2.4 formulas against worked numbers; the baseline fit removes a linear drift exactly; the
  integral of a pure exponential equals `(y0 - y_inf) * tau`.
- **Properties:** X and Y do not depend on pulse energy; a baseline offset and slope do not change X and Y; a pulse
  that never rises reports `kind` 1; every saturated quadrant reports `kind` 2; no input sequence panics.
- **Simulator:** the thermal front end in `align-sim`, so that the closed-loop tests can run the whole chain from a
  mirror tilt to a SENSOR message and back, with tau, noise, and drift as scenario parameters.
- **Board:** the simulated front end on the development board, over the self-test CAN loopback, then the real
  front end once it exists.

## 7. Milestones

They follow M4 and do not block M5: the coordinator can be built against the simulated sensor.

| Milestone | Contents | Needs from the owner |
|---|---|---|
| S0 | Protocol: SHOT, `kind` values, sum scaling, in `align-proto` with round-trip tests | Sign-off on section 3 |
| S1 | `sensor-logic`: buffer, baseline, integration, three-sample fit, checks, state machine, console parser; the thermal model in `align-sim` | Nothing |
| S2 | `firmware/sensor` on the development board with the simulated front end: CAN, heartbeat, status, console, flash record | Nothing (provisional pins) |
| S3 | The real front end driver | Open item 6: detector type, ADC and its interface, gain, noise, tau, beam radius, pins |
| S4 | Commissioning on the machine: `tau_cal`, `S_ref`, and the choice of integration or fit | The sensor built and mounted |

## 8. Questions for the owner

1. Do you sign off on the SHOT message and the SENSOR field meanings in section 3?
2. Is anyone else writing the sensor firmware, or does it belong in this repository?
3. What do you know so far about the detector (thermopile, pyroelectric, or photodiode behind a converter) and how it
   connects (an ADC chip over SPI or I2C, or the ESP32-C6's own ADC)? Even a part number narrows section 4.

**Recommendation:** approve section 3 and start S0 and S1 now. They need no hardware, they settle the interface the
coordinator (M5) is built against, and the simulated front end lets S2 run on the development board.
