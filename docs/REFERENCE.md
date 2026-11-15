# Auto Mirror Alignment Reference: CO2 Laser, ESP32 + Embassy (Rust)

The architecture works, but four settled decisions need to change before firmware starts: use the ESP32-C6 as the node MCU (the C3 has no PCNT encoder counter), measure at three gantry positions using differences rather than an absolute near/far 4-vector, power the DRV887x drivers from VM of 6 V or more (they do not operate at 3 to 5 V), and make the laser interlock fail-safe in the laser path itself.

**TL;DR**

- **Hardware:** ESP32-C6 nodes (PCNT, TWAI, MCPWM on one chip, stable RISC-V Rust) with DRV8876 drivers at VM = 6 V and PWM duty capped for an effective 3 to 5 V. Size current limits from the Pololu 2 kg·cm instantaneous-torque guidance, not the theoretical stall figures.
- **Solver:** 4x4 empirical Jacobian, damped least squares, step gain 0.6, Broyden updates, on a three-position difference vector (Y near to far at fixed X, then X near to far). This cancels the unknown head offset and gives a block-triangular, well-conditioned matrix. A two-position absolute vector cannot tell offset from angle.
- **Ruida:** for V1, sense L-ON or LPWM through an optocoupler, start jobs via FootSW, detect job end from CN1 Status. The UDP protocol (port 50200, swizzle magic 0x88, ACK 0xCC arriving swizzled as 0xC6) is well reverse-engineered for 644xG boards but unverified on Mod5 firmware. First test is read-only: `DA 00 05 7E`.

---

## Table of Contents

0. Conventions and project-wide flags
1. Optics and geometry math
2. Quadrant sensor math
3. Calibration and solver
4. Motor control logic
5. Embassy and esp-hal reference
6. Ruida protocol and I/O reference
7. Safety logic
8. Mechanical facts and prior art
9. To-measure checklist, constants, glossary
10. Caveats

---

## 0. Conventions and project-wide flags

Tags: **VERIFIED** (named primary or vendor source), **CALCULATED** (formula and inputs shown), **UNVERIFIED** (secondary source or judgment; must be measured or tested).

Code uses two-space indentation. Repo root `rustfmt.toml`:

```toml
tab_spaces = 2
```

Units go in identifier names: `_counts`, `_um`, `_urad`, `_ms`, `_ma`, `_norm`.

| # | Flag | Severity | Status |
|---|---|---|---|
| F1 | ESP32-C3 has no PCNT.\[1\]\[2\]\[3\] GPIO-interrupt decoding loses counts during flash writes or long critical sections. | High | VERIFIED: Espressif ESP-FAQ ("What should I do if ESP32-C3 doesn't support PCNT?"); ESP-IDF SoC capability table |
| F2 | DRV8876/DRV8874 VM range is 4.5 to 37 V, UVLO rising 4.45 V typ (4.6 V max). "Run motors at 3 to 5 V" is impossible as a supply. Use VM = 6 V and cap duty. | High | VERIFIED: TI DRV8876 datasheet SLVSDS7B |
| F3 | Absolute readings at two positions cannot separate head offset from the two leg angles; the solve can converge to a misaligned state. Use three-position differences (1.5). | High | CALCULATED |
| F4 | A one-shot on the fire-sense line is not fail-safe: a broken sense wire reads as "laser idle". Add a force-guided relay that opens the laser enable loop whenever motor permit is asserted (7.1). | High | CALCULATED |
| F5 | RF tickle or standby pulses on LPWM would retrigger a 100 ms one-shot forever. Fire detection must reject tickle by pulse width. | Medium | Ruida "Laser Standby Pulse/Frequency" parameters exist: VERIFIED, EduTech Wiki;\[4\] tickle widths UNVERIFIED |
| F6 | Pololu stall numbers are theoretical; stalls could damage the gearbox.\[5\] | Medium | VERIFIED: Pololu #3069 |
| F7 | In esp-hal, TWAI, PCNT, MCPWM and ADC are all partial (⚒️) and require the `unstable` feature.\[6\] Pin versions. | Medium | VERIFIED: esp-hal peripheral support table (lib.rs); esp-hal docs for `esp_hal::pcnt`\[7\] |

---

## 1. Optics and geometry math

### 1.0 Pitfalls

- The factor of 2 holds only for tilt about an axis perpendicular to the plane of incidence. At 45 degree incidence, out-of-plane tilt deflects by 2·cos(45°) = √2.
- Four tilt actuators control angles only. They cannot remove a constant beam offset at the head (set mechanically by tube or M1 placement). Log it; do not chase it.
- Do not hard-code a theoretical Jacobian. The empirical Jacobian absorbs mount orientation, signs, cross-coupling and sensor slope.

### 1.1 Single-screw chain (CALCULATED)

```
screw_travel_m  = counts / CPR / G * p
mirror_tilt_rad = screw_travel_m / L
beam_angle_rad  = k * mirror_tilt_rad      k = 2 (in-plane) or sqrt(2) (out-of-plane, 45 deg)
beam_disp_m     = beam_angle_rad * D
=> beam_disp_m  = counts * p * k * D / (CPR * G * L)
```

| Symbol | Meaning | Placeholder | Status |
|---|---|---|---|
| CPR | Motor-shaft counts per rev, quadrature | 12 (Pololu); 28 (7 PPR x4); 12 (3 PPR x4) | Pololu 12 CPR: VERIFIED (Pololu encoder pages);\[8\]\[9\] 7/3 PPR: VERIFIED as vendor claims (Waveshare, Kegu, NFP)\[10\]\[11\]\[12\] |
| G | Gear ratio | 297.92 | VERIFIED: Pololu #3069\[5\] |
| p | Screw lead | 0.5e-3 m | UNVERIFIED: measure |
| L | Lever arm, screw to pivot line | 25e-3 m | UNVERIFIED: measure |
| D | Mirror to measurement plane | 0.1 to 1.5 m | UNVERIFIED: measure |

Worked example (k = 2, D = 1 m):

| Quantity | 12 CPR | 28 CPR |
|---|---|---|
| Screw travel per motor rev | 1.678 um | 1.678 um |
| Screw travel per count | 0.140 um | 0.060 um |
| Mirror tilt per count | 5.59 urad | 2.40 urad |
| Beam displacement per count at 1 m | 11.2 um | 4.8 um |
| Beam displacement per um of screw at 1 m | 80 um | 80 um |

This confirms the project figures (1.7 um/rev, 40 urad tilt and 80 urad beam per um). Resolution is not the limit; stick-slip and backlash are.

### 1.2 The 45 degree factor (CALCULATED)

| Tilt axis | Deflection | At 45° |
|---|---|---|
| Perpendicular to plane of incidence | 2δ, in-plane | 2δ |
| In mirror surface, in plane of incidence | 2δ·cos(i), out of plane | 1.414δ |

Both flying-optics turns are horizontal, so per mirror the screw with a vertical tilt axis steers horizontally (k = 2) and the other steers vertically (k = √2). Which L-pattern screw is which is ASSUMED; verify by poking.

### 1.3 Flying-optics model (CALCULATED)

Leg 1: M1 (frame) to M2 (Y gantry), length Ly. Leg 2: M2 to head, length Lx. Per transverse axis:

```
s(Ly, Lx) = s0 + e1*Ly + e2*Lx
d e1/d q_M1 = k1*c1     d e2/d q_M1 = k1*c1     (M1 tilt also changes the angle into M2)
d e2/d q_M2 = k2*c2     d e1/d q_M2 = 0
```

Moving along one leg changes only that leg's lever arm, so a difference of two readings depends only on angle. This is the basis of the classic method.

### 1.4 Measurement vector: three positions, differences (firm)

| Point | Gantry Y | Head X |
|---|---|---|
| A | Near (close to M1) | Near (close to M2) |
| B | Far | Near |
| C | Far | Far |

```
m = [hB-hA, vB-vA, hC-hB, vC-vB]^T = [e1h*dY, e1v*dY, e2h*dX, e2v*dX];   target m = 0
J_axis (M1, M2) = [ k1*c1*dY      0       ]
                  [ k1*c1*dX   k2*c2*dX   ]
```

| dY (mm) | dX (mm) | cond(J_axis), k1c1 = k2c2 |
|---|---|---|
| 500 | 500 | 2.62 |
| 500 | 800 | 3.54 |
| 50 | 800 | 32 |
| → 0 | any | → infinity |

Conditioning depends on dX/dY; noise amplification goes as about 1/span. Use full travel minus margins. If near and far are close, differences fall into noise and the damped solver stalls.

The absolute vector `[hN, vN, hF, vF]` is rejected: with s0 ≠ 0 it has a one-parameter family of solutions that pass through center at N and F but sweep off-center at (Yf, Xn). Keep `hA, vA` as a health metric; if |s0| exceeds about 30% of the sensor linear range, flag "mechanical centering required".

### 1.5 Sequential versus joint (firm)

Use the joint 4x4 solve on the difference vector. Because J is block-triangular, it equals the classic sequential method plus cross-coupling correction at the same shot cost. Fallback if cond(J) > 50: sequential 2x2 (M1 from rows 1 and 2, then M2 from rows 3 and 4).

---

## 2. Quadrant sensor math

### 2.0 Pitfalls

- The textbook slope assumes TEM00. A multimode glass-tube beam is not Gaussian and changes with power and warm-up. Control in normalized units; let the Jacobian absorb the slope.
- Thermal detectors have time constants and baseline drift. Ratios cancel pulse energy, not offsets, so subtract baselines per quadrant first.
- Gap-correction papers model ideal beams (for example the Laguerre-Gaussian method in Sensors 2018, 18(11):4007); treat them as a guide.\[13\]\[14\]

### 2.1 Normalized position (VERIFIED form: conventional QD formula, Sensors 2018 QD paper)\[15\]\[16\]

```
  B | A          S = A + B + C + D
  --+--          X = ((A + D) - (B + C)) / S
  C | D          Y = ((A + B) - (C + D)) / S
```

A to D are baseline-subtracted, integrated quadrant signals.

### 2.2 Gaussian slope and linear range (CALCULATED)

For intensity ∝ exp(−2r²/w²), zero gap, large detector:

```
X = erf(sqrt(2) * x0 / w)        x0 = (w / sqrt(2)) * erfinv(X)
dX/dx0 at center = 1.596 / w     x0 ≈ 0.627 * w * X (small signal)
```

| Range | Max abs X | Max abs x0 | Linearity error |
|---|---|---|---|
| 5% | 0.42 | 0.28 w | 5% |
| 10% | 0.57 | 0.40 w | 10% |
| erfinv-corrected | 0.90 | 0.82 w | model-limited |

Solver accepts |X|, |Y| ≤ 0.6; beyond that, use sign and coarse steps only.

### 2.3 Gap and validity checks

Each gap strip of width g blocks a fraction of about 0.80·g/w (CALCULATED). Require g ≤ 0.1 w in the sensor design. Thresholds below are UNVERIFIED starting points.

| Check | Rule | Failure meaning |
|---|---|---|
| Sum present | S ≥ 0.3·S_ref | Off sensor, clipped, or no fire |
| Not saturated | max quadrant < 0.9 full scale | Gain too high |
| Quadrant sanity | each ≥ −3σ_noise | Baseline error |
| In range | abs X, abs Y ≤ 0.9 | At edge |
| Repeatability | two shots agree within 3σ | Creep or mode hop |
| Energy | S within ±30% of previous at same power | Misfire or upstream clipping |

### 2.4 Thermal detectors (CALCULATED)

```
y(t) = y_inf + (y0 - y_inf) * exp(-t / tau)
three equal-spaced samples: y_inf = (y1*y3 - y2^2) / (y1 + y3 - 2*y2)
                            tau   = -Δ / ln((y3 - y2) / (y2 - y1))
```

Reject if the denominator is under about 5σ or tau is outside [0.3, 3]·tau_cal.

Preferred for short dots: energy integration. ∫(y − baseline)dt over the full decay equals gain·E, and X, Y cancel E.

1. Sample 0.5 s before fire for baseline mean and slope.
2. From the fire-line timestamp, integrate each quadrant minus the extrapolated baseline.
3. Stop at 5·tau or when S returns within 2σ of baseline.
4. Compute X, Y, S and run validity checks.

Thermopile tau is typically tens of ms to seconds (UNVERIFIED; measure per quadrant). Sample at least 10 per tau.

---

## 3. Calibration and solver

### 3.0 Pitfalls

- Pokes too small are noise-dominated; too large leave the linear range.
- Every poke must finish in the standard direction or backlash corrupts J.
- Normal equations square the condition number; acceptable for a damped 4x4, but monitor cond(J) via SVD.

### 3.1 Empirical Jacobian (firm)

1. Move to q0 with standard approach; measure m0 (shots A, B, C).
2. For each axis j: move +Δq_j, measure m_j, set `J[:, j] = (m_j − m0) / Δq_j`, return with overshoot-and-approach.
3. Reject if cond(J) > 50 or any column norm < 5·σ_m / Δq_j.
4. Persist J, q0, cond, timestamp.

Poke size: `dq = 0.25 / ((1.596 / w) * beam_um_per_count_at_D)`. Example w = 4 mm, 11.2 um/count gives about 56 counts. Clamp to [3 × backlash, 500]. Average 2 shots only if repeatability σ > 0.01 normalized. Cost: 15 shots; poking M2 with only B and C saves 2.

### 3.2 Solver (firm)

```
r = m - 0
dq = alpha * (J^T J + lambda^2 I)^-1 J^T (-r),  alpha = 0.6
lambda = 0.05 * sigma_max(J)   (x3 after a failed step, /3 after a good one)
clamp dq_j to +/- 4 x poke size
Broyden: J <- J + ((dm - J dq) dq^T) / (dq^T dq)   skip if norm(dq) < 2 x min reliable move
```

| Condition | Action |
|---|---|
| norm_inf(m) ≤ 0.02 | Success; store last-known-good |
| Residual reduced < 30% | Raise lambda, retry once, then recalibrate |
| Iterations > 4 | Recalibrate once, 4 more, then Fault |
| cond(J) > 50 after update | Discard update, recalibrate |
| Validity check fails | Coarse acquisition (3.5) |
| Step would exceed soft limit | Fault, manual pre-alignment |

### 3.3 no_std crates

| Crate | Role | Status |
|---|---|---|
| nalgebra (default-features = false, features = ["libm"]) | Matrix4, Cholesky, SVD | Disabling std documented in nalgebra user guide: VERIFIED.\[17\] Static-size SVD without alloc for your version: UNVERIFIED, test-compile |
| numeris | no_std LU, QR, SVD, no heap | VERIFIED: lib.rs listing\[18\] |

Use nalgebra `Matrix4<f32>`: Cholesky for the damped solve, SVD only for cond. Scale dq to hundreds of counts so J entries are 0.01 to 1.

```rust
use nalgebra::{Matrix4, Vector4};

pub fn dls_step(
  j: &Matrix4<f32>,
  r: &Vector4<f32>,
  lambda: f32,
  alpha: f32,
  dq_max: &Vector4<f32>,
) -> Option<(Vector4<f32>, f32)> {
  let s = j.svd(false, false).singular_values;
  if !(s.min() > 0.0) {
    return None;
  }
  let jt = j.transpose();
  let a = jt * j + Matrix4::identity() * (lambda * lambda);
  let mut dq = a.cholesky()?.solve(&(jt * (-r))) * alpha;
  for i in 0..4 {
    dq[i] = dq[i].clamp(-dq_max[i], dq_max[i]);
  }
  Some((dq, s.max() / s.min()))
}

pub fn broyden_update(j: &mut Matrix4<f32>, dq: &Vector4<f32>, dm: &Vector4<f32>) {
  let d = dq.dot(dq);
  if d >= 1.0 {
    *j += (dm - *j * dq) * dq.transpose() / d;
  }
}
```

### 3.4 Backlash and standard approach

Standard direction (+) is the screw advancing against the springs.

```text
fn approach(axis, target):
  if last_dir == Plus and target >= pos: move_to(target)
  else: move_to(target - OVERSHOOT); move_to(target)
  last_dir = Plus
```

OVERSHOOT = max(2 × measured backlash, 60) counts at 12 CPR. Initial guess: 1 to 3 degrees of output backlash gives 10 to 30 counts (CALCULATED). No published N20 backlash data was found (UNVERIFIED).

Backlash via beam: approach q0 in +, measure 3 times for σ. Then go to q0 − b and back to q0 in + without overshoot, for b = 4, 8, 16, 32, 64. The b where the reading shifts by more than 3σ brackets backlash plus stick-slip; bisect once.

Hysteresis: cycle ±2 poke sizes in 8 steps per half-cycle, one shot per step. Loop width is effective hysteresis; a step after reversal is seat stick-slip; repeated-shot drift means a longer settle delay.

### 3.5 Coarse acquisition

1. Return to LastGood counts and shoot.
2. Archimedean spiral on M2 (radial step = half the usable sensor radius in counts), one low-power shot per point, cap 25.
3. Same on M1 with the head at A, cap 25.
4. Fault: "manual pre-alignment required".

### 3.6 Persistence

| Record | Content | Written when |
|---|---|---|
| AxisPos[k] | pos i32, last_dir u8, seq u32, crc | After every move, motors stopped |
| Calib | J (16 f32), q0, cond, seq | After calibration |
| LastGood | q (4 i32), residual, seq | After convergence |
| Sensor | tau_cal, S_ref | Commissioning |

---

## 4. Motor control logic

### 4.0 Pitfalls

- Some secondary sources wrongly list PCNT on the C3 (circuitlabs.net).\[19\] Ignore them.
- PCNT unit counts: an ESP32Encoder README says the S3 has only 2 units, but Espressif's own datasheets contradict it. The ESP32-C6 Series Datasheet v1.6 (§4.2.1.5) lists "Four independent pulse counters with two channels each", and the ESP32-S3 Series Datasheet (§3.5.9) lists "Four independent pulse counters (units) that count from 1 to 65535" (VERIFIED). The C6 design needs only 2 or 3.
- IPROPI error at low current is ±7.5 mA absolute below 0.15 A (VERIFIED, DRV8876 datasheet), so N20 running currents of 50 to 150 mA are known only to about ±5 to 15%.\[20\]

### 4.1 Node MCU (firm: ESP32-C6)

| Need | ESP32-C3 | ESP32-C6 | ESP32-S3 |
|---|---|---|---|
| PCNT | None | Yes | Yes\[3\]\[21\] |
| TWAI | 1 | 2 controllers (ESP32-C6 datasheet)\[22\] | Yes\[23\] |
| MCPWM | None | Yes | Yes |
| Toolchain | Stable RISC-V | Stable RISC-V | Xtensa fork via espup |
| esp-hal TWAI / PCNT / MCPWM / ADC | ⚒️ / none / none / ⚒️ | ⚒️ / ⚒️ / ⚒️ / ⚒️ | ⚒️ / ⚒️ / ⚒️ / ⚒️ |

VERIFIED: ESP-IDF SoC table and esp-hal peripheral support table.\[7\]\[24\] Use the C6 for nodes and coordinator: one target, stable Rust, hardware counting immune to flash stalls, and Wi-Fi for later Ruida UDP.

### 4.2 Encoder edge rates (CALCULATED from Pololu #3069: 110 RPM out at 6 V)

Motor shaft = 110 × 297.92 = 32,771 RPM = 546 rev/s.\[5\]

| Effective V | Motor rev/s | 12 CPR edges/s | 28 CPR edges/s | Min interval (28) |
|---|---|---|---|---|
| 6 | 546 | 6,554 | 15,293 | 65 us |
| 4.5 | 410 | 4,916 | 11,470 | 87 us |
| 3 | 273 | 3,277 | 7,647 | 131 us |

Two axes at 28 CPR is about 30,000 interrupts/s on a C3: feasible on paper, fragile in practice (a flash erase or critical section over 65 us drops counts). On the C6, PCNT counts in hardware and a task polls every 1 to 5 ms. Enable the PCNT glitch filter (5 us is used in one ESP32Encoder project; UNVERIFIED here).\[25\]

### 4.3 DRV8876 facts (VERIFIED: TI DRV8876 datasheet SLVSDS7B)\[20\]

| Item | Value |
|---|---|
| VM / UVLO rising | 4.5 to 37 V / 4.45 V typ |
| Peak current, RDS(on) HS+LS | 3.5 A, 700 mΩ\[20\]\[26\] (DRV8874: 6 A, 200 mΩ) |
| A_IPROPI | 1000 uA/A (DRV8874: 450 uA/A; its example uses 455)\[20\]\[27\] |
| A_ERR | ±7.5 mA (< 0.15 A); ±5% (0.15 to 0.5 A); ±4% (0.5 to 2 A, PWP) |
| Trip | ITRIP · A_IPROPI = VVREF / RIPROPI; VREF 0 to 3.6 V\[28\] |
| tOFF / tBLK / tDEG | 25 us / 1.1 us / 0.6 us |
| OCP | 3.5 to 5.5 A, retry 2 ms or latched |
| PWM input | 0 to 100 kHz |
| nSLEEP | Low = sleep, Hi-Z; 100 kΩ internal pulldown; tWAKE 1 ms |
| nFAULT | Open-drain; low on UVLO, CPUV, OCP, TSD; in cycle-by-cycle mode also low while chopping |
| IPROPI | Senses low-side current only; coast current not sensed\[29\] |

| PMODE | Mode | Essentials |
|---|---|---|
| Low | PH/EN | EN = 0 brake; EN = 1 with PH sets direction |
| High | PWM | 00 coast, 01 reverse, 10 forward, 11 brake |
| Hi-Z | Half-bridge | Current regulation disabled |

| IMODE | Chopping | OCP | nFAULT reports |
|---|---|---|---|
| GND | Fixed off-time | Retry | OCP only |
| 20 kΩ | Cycle-by-cycle | Retry | Chopping and OCP |
| 62 kΩ | Cycle-by-cycle | Latched | Chopping and OCP |
| Hi-Z | Fixed off-time | Latched | OCP only |

Firm choices:

- **PMODE = GND (PH/EN).** PWM on EN; EN = 0 between pulses brakes and keeps IPROPI valid.
- **IMODE = 20 kΩ.** nFAULT low during drive means current limit reached (hardware stall flag); low while EN = 0 means a real fault (datasheet method).\[20\]
- **VM = 6.0 V**, duty capped at 0.5 to 0.8.
- **DRV8876 over DRV8874**: higher A_IPROPI doubles volts per amp at these small currents.

DRV8833 comparison (VERIFIED, TI DRV8833 datasheet): VM 2.7 to 10.8 V; I_chop = V_TRIP / R_ISEN, V_TRIP 200 mV typ (160 to 240); "If current control is not needed, the xISEN pins should be connected directly to ground", which is why grounded-sense breakouts have no limit; no current output to the MCU.\[30\]\[31\]\[32\]

### 4.4 Current limit sizing (CALCULATED)

From Pololu HPCB 6 V 298:1 (3.4 kg·cm = 0.333 N·m at 1.5 A, 0.15 A no load):\[5\]

```
K_out = 0.333 / (1.5 - 0.15) = 0.247 N·m/A     T_out ≈ K_out * (I - I_nl)
F = 2*pi*eta*T / p     eta = tan(lambda) / tan(lambda + phi')
lambda = atan(p / (pi*d_m))     phi' = atan(mu / cos 30°)
```

Pololu: "The recommended upper limit for instantaneous torque is 2 kg-cm (25 oz-in) for all the gear ratios" (VERIFIED as reproduced on Pimoroni's Pololu listing).\[33\] Assumed M6 × 0.5, d_m 5.68 mm, mu 0.15 gives eta ≈ 0.14 (ASSUMED).

| Max torque | I_trip | Force (eta 0.14) | Force (eta 0.5) |
|---|---|---|---|
| 0.196 N·m (Pololu limit) | 0.94 A | 345 N | 1,232 N |
| 0.10 N·m | 0.55 A | 176 N | 628 N |
| 0.05 N·m (start here) | 0.35 A | 88 N | 314 N |

The project's 500 to 1000 N estimate is plausible only at near-ideal efficiency.

Settings: RIPROPI = 6.2 kΩ; VREF = 6.2 kΩ × 0.35 A × 1 mA/A = 2.17 V. After measuring I_run, I_trip = clamp(3·I_run, I_run + 0.1 A, 0.35 A). Add 10 nF on IPROPI if chopping triggers early (datasheet guidance).\[20\]\[29\]

### 4.5 Move controller (firm)

| Parameter | Start value | Status |
|---|---|---|
| PWM frequency | 20 kHz | UNVERIFIED acoustics and heating |
| Cruise duty | 0.6 (≈ 3.6 V) | UNVERIFIED |
| Decel zone | Last 24 counts at creep duty | UNVERIFIED |
| Stop | Brake when remaining ≤ learned coast | UNVERIFIED |
| Tolerance | ±3 counts; record actual, do not chase | Firm |
| Settle | 200 ms before any shot | UNVERIFIED |
| Stall | No count change for max(50 ms, 5 × expected edge interval) while driven, or nFAULT during drive | Firm |
| Caps | ≤ 2000 counts, ≤ 2 s per move | UNVERIFIED |

Endpoint error only needs to be repeatable. Expect a minimum reliable move near one motor rev (12 counts); measure it.

### 4.6 ADC for IPROPI

esp-hal ADC eFuse calibration is supported on C3 and C6; DMA continuous mode, IIR filter and threshold monitor are missing (VERIFIED, esp-hal issue #6747).\[34\] Use highest attenuation, oversample 16 to 64, keep the signal within about 0.1 to 2.5 V (UNVERIFIED C6 limits), and calibrate end-to-end against a series ammeter to absorb A_ERR.

---

## 5. Embassy and esp-hal reference

### 5.0 Pitfalls

- **esp-hal-embassy and esp-wifi are gone.** esp-hal-embassy merged into esp-rtos; esp-wifi was replaced by esp-radio (VERIFIED, esp-hal 1.0.0 release notes).\[35\]\[36\]
- **API churn.** esp-hal 1.1 and esp-rtos 0.3 (April 2026) changed `esp_rtos::start` to take `SoftwareInterrupt<'static, 0>` plus the timer and replaced the `Efuse` struct with free functions (VERIFIED only via a third-party dependency PR, sam-ruff/walkie-textie-firmware).\[37\] esp-rtos 0.4.0 is current, minimum Rust 1.95 (VERIFIED, crates.io).\[38\] Pin versions.
- **No TWAI single-shot.** `transmit()` retries until success or error-passive abort (VERIFIED, esp-hal issue #6781).\[39\] Always wrap transmits in a timeout.
- **Flash writes stall the CPU.** Persist only after stop and brake.

### 5.1 Project setup

| Item | Recommendation | Status |
|---|---|---|
| HAL | esp-hal 1.x, `unstable` feature for TWAI, PCNT, MCPWM, ADC | VERIFIED |
| Runtime | esp-rtos with `embassy` feature (executor and embassy-time driver) | VERIFIED: Rust on ESP Book; Embassy Book\[40\]\[41\] |
| Template | esp-generate | UNVERIFIED flags |
| Target (C6) | Stable Rust, `riscv32imac-unknown-none-elf` | UNVERIFIED |
| Flash and debug | espflash; probe-rs over USB-Serial-JTAG | UNVERIFIED |
| Logging | defmt or esp-println | UNVERIFIED |
| Wi-Fi | esp-radio with embassy-net | VERIFIED\[42\] |

### 5.2 Node tasks and primitives (firm)

| Task | Inputs | Outputs |
|---|---|---|
| can_rx | TWAI | Per-axis Channel, inhibit Watch |
| can_tx | Status Channel | TWAI with timeout |
| axis[0..3] | Command Channel, PCNT, ADC, nFAULT | Status, persist request |
| safety (interrupt executor) | Inhibit pin, heartbeat age, nFAULT | STOP Signal, ARM pulse |
| persist | Persist Channel | Flash |

| Need | Primitive |
|---|---|
| Per-axis commands | bounded `Channel<Raw, T, N>` |
| Stop everything | `Signal` |
| Inhibit to many readers | `Watch` |
| Timeouts / periodic | `with_timeout` / `Ticker` |

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as Raw;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration};

#[derive(Clone, Copy)]
pub enum AxisCmd {
  MoveRel { seq: u8, delta_counts: i32, finish_plus: bool },
  Stop,
}

pub static AXIS_CMD: [Channel<Raw, AxisCmd, 2>; 3] = [const { Channel::new() }; 3];

#[embassy_executor::task(pool_size = 3)]
async fn axis_task(axis: usize) {
  loop {
    if let AxisCmd::MoveRel { delta_counts, finish_plus, .. } = AXIS_CMD[axis].receive().await {
      let r = with_timeout(Duration::from_millis(2000), do_move(axis, delta_counts, finish_plus)).await;
      if r.is_err() {
        // brake, report Timeout
      }
    }
  }
}

async fn do_move(_axis: usize, _delta: i32, _plus: bool) {}
```

Signatures follow embassy-sync and embassy-time conventions; UNVERIFIED against your pinned versions.

### 5.3 TWAI

- esp-hal: blocking and async TX/RX, error counters, bus-off recovery, self-test and listen-only, timing and acceptance filters implemented and hardware-tested; single-shot, CLKOUT and bus-off indicator output missing (VERIFIED, issue #6781).\[39\]
- Bus-off at TEC ≥ 256, recovery required (VERIFIED, ESP-IDF TWAI docs for C6).\[43\]\[44\]\[45\] Policy: brake all, wait 100 ms, recover, re-announce; motors stay stopped until re-armed.
- Use 250 kbit/s, sample point about 87.5%, for a 2 to 3 m drag-chain bus (UNVERIFIED choice). SN65HVD230 at 3.3 V, 120 Ω at both ends only, twisted CAN H/L, shared ground.

### 5.4 CAN protocol (firm proposal)

11-bit ID: `[10:7] class | [6:3] node_id | [2:0] axis`. Lower ID wins arbitration.

| Class | Name | Dir | DLC | Payload (LE) |
|---|---|---|---|---|
| 0x0 | ESTOP | coord → all | 1 | reason u8 |
| 0x1 | INHIBIT | coord → all | 2 | inhibited u8, source u8 |
| 0x2 | FAULT | node → coord | 4 | axis u8, code u8, detail u16 |
| 0x3 | MOVE_REL | coord → node | 7 | seq u8, axis u8, delta i32, flags u8 |
| 0x4 | STOP | coord → node | 1 | reason u8 |
| 0x5 | MOVE_DONE | node → coord | 8 | seq, axis, result, pad, pos i32 |
| 0x6 | STATUS | node → coord | 8 | axis u8, flags u8, pos i32, current_ma u16 |
| 0x7 | SENSOR | sensor → coord | 8 | seq u8, kind u8, x_q15 i16, y_q15 i16, sum u16 |
| 0x8 | HEARTBEAT | both | 6 | uptime_ms u32, state u8, fw u8 |
| 0x9 | CONFIG | coord → node | 8 | key u8, axis u8, pad u16, value i32 |

STATUS flags: bit0 moving, bit1 at_target, bit2 stalled, bit3 nfault, bit4 soft_limit, bit5 inhibited, bit6 persisted, bit7 armed. Coordinator heartbeat every 100 ms; nodes brake and disarm after 300 ms without one. MOVE_REL is idempotent by seq. Use manual fixed layouts, not serde.

```rust
pub fn encode_move_rel(seq: u8, axis: u8, delta: i32, plus: bool) -> [u8; 7] {
  let d = delta.to_le_bytes();
  [seq, axis, d[0], d[1], d[2], d[3], plus as u8]
}
```

### 5.5 Flash persistence

esp-storage with sequential-storage `map` on a dedicated partition of at least 4 sectors (UNVERIFIED API; ekv is the alternative). Wear (CALCULATED, assuming 100k cycles, UNVERIFIED): 32-byte records give 128 per 4 KiB sector; 400 writes per day over 4 sectors is 0.8 erases per sector per day, over 300 years. Each record carries seq and CRC; boot takes the highest valid seq.

### 5.6 Coordinator networking

Wi-Fi: esp-radio plus embassy-net UDP (VERIFIED naming).\[42\] Wired: W5500 via embassy-net-wiznet (UNVERIFIED crate status). Prefer wired Ethernet to a dedicated Ruida switch.

---

## 6. Ruida protocol and I/O reference

### 6.0 Pitfalls and disagreements

- **Nothing here is verified on RDC6445G Mod5 firmware.**
- **Checksum basis:** the EduTech Wiki says "The checksum is calculated preswizzle"; a MeerK40t author forum post says "calculated unswizzled".\[4\]\[46\] Both are ambiguous; the first test resolves it.
- **Card ID:** wiki says the 05 7E reply is 0x65006500; MeerK40t maps 0x65106510 to RDC6442S.\[4\]\[47\] Do not hard-code.
- **Replies are swizzled:** ACK 0xCC arrives as 0xC6 with magic 0x88 (VERIFIED: jnweiger protocol.md "0xc6 if all is well"; algebra checked).\[48\]\[49\]
- **No arbitration:** "There is no header, arbitration, or handshaking. Anything that is interpreted as correct will execute." (VERIFIED, EduTech Wiki).\[4\]

### 6.1 Transport (VERIFIED: EduTech Wiki; MeerK40t rdjob.py, controller.py)

| Item | Value |
|---|---|
| Main port | Listens UDP 50200, replies from 40200\[4\] |
| Key port | 50207 / 40207, no swizzle, `A5 50 xx` key down, `A5 51 xx` key up, 0xCE keep-alive\[4\] |
| Packet | [csum_hi, csum_lo] + swizzled payload; csum = low 16 bits of byte sum\[4\] |
| Replies | 0xCC ACK, 0xCF NAK (resend), 0xCD ERR, 0xCE ENQ\[4\]\[47\] |
| Size | Wiki 1474 bytes max, modern cuts about 1000 at command boundaries\[4\] |
| ACK timeout | About 4 s; "The first chunk will retry other chunks will simply fail."\[4\] |
| Polling | MeerK40t polls every 0.2 s; timeout 1 s normal, 40 s homing\[50\] |
| Keep-alive on 50200 | None documented |

Swizzled with 0x88 (CALCULATED, matches jnweiger): ACK → 0xC6, ERR → 0x46, NAK → 0x48.\[48\]

### 6.2 Swizzle (VERIFIED: EduTech Wiki, MeerK40t)

Magic: 0x88 for 644XG, 644XS, 320, 633X, 654XG and default; 0x11 for 634XG; 0x38 for RDL9635. Algorithm: swap bit 7 and bit 0, XOR magic, add 1.\[4\]

```rust
pub const fn swizzle(mut b: u8, magic: u8) -> u8 {
  b ^= b >> 7;
  b ^= b << 7;
  b ^= b >> 7;
  (b ^ magic).wrapping_add(1)
}

pub const fn unswizzle(b: u8, magic: u8) -> u8 {
  let mut b = b.wrapping_sub(1) ^ magic;
  b ^= b >> 7;
  b ^= b << 7;
  b ^= b >> 7;
  b
}

/// Checksum over swizzled bytes if `over_swizzled`, else plain. Basis UNVERIFIED: test both.
pub fn build_packet(plain: &[u8], magic: u8, over_swizzled: bool, out: &mut [u8]) -> usize {
  let mut sum: u16 = 0;
  for (i, &p) in plain.iter().enumerate() {
    let s = swizzle(p, magic);
    out[2 + i] = s;
    sum = sum.wrapping_add(if over_swizzled { s } else { p } as u16);
  }
  out[0] = (sum >> 8) as u8;
  out[1] = sum as u8;
  plain.len() + 2
}
```

### 6.3 Encodings (VERIFIED: EduTech Wiki; MeerK40t decodeu35)

| Type | Bytes | Unit |
|---|---|---|
| Abscoord | 5 (35-bit, 7 bits per byte, MSB first) | um\[4\]\[47\] |
| Relcoord | 2 (14-bit signed) | um\[4\] |
| Power | 2 (14-bit) | 100/2^14 percent\[4\] |
| Time / Frequency | 5 | us / Hz\[4\] |

Bytes ≥ 0x80 start commands, so data bytes are always < 0x80.\[4\]

```rust
pub fn dec35(d: &[u8; 5]) -> u32 {
  d.iter().fold(0u32, |acc, &b| (acc << 7) | (b as u32 & 0x7F))
}
```

### 6.4 Command subset (VERIFIED: EduTech Wiki; MeerK40t rdjob.py)

| Bytes | Meaning |
|---|---|
| DA 00 hi lo | GET; reply `DA 01 hi lo v0(5) v1(5)`\[4\]\[48\] |
| DA 00 04 00 | Status. Bits: 0x01000000 moving, 0x02 part end, 0x01 job running\[47\] |
| DA 00 04 21 / 04 31 | Current X / Y (um); wiki labels them "Axis Preferred Position 1/2"\[4\]\[47\] |
| DA 00 05 7E / 05 7F | Card ID / mainboard version\[4\] |
| D9 00/01 opt abs | Rapid X / Y (opt 2 = no origin, no light)\[4\] |
| 88 absX absY | Move absolute\[4\] |
| D8 00 / 01 / 02 / 03 | Start / Stop / Pause / Restore\[47\] |
| 50207 key values | 05 Pulse, 06 Start/Pause, 09 Stop, 01/02 X−/X+, 03/04 Y+/Y−\[4\] |

### 6.5 Mod5 first test (firm)

1. Close LightBurn or use another host; a second app on the LightBurn PC cannot also bind port 40200 (KerfDesk PR #1707, third-party).\[51\]
2. Send `DA 00 05 7E`, magic 0x88, checksum over swizzled bytes.
3. Expect 0xC6, then a datagram unswizzling to `DA 01 05 7E ...`. On 0x48, retry with plain checksum. On silence, check IP and port before touching magic.
4. Read 04 00, 04 21, 04 31 while jogging from the panel; confirm um units and sign (MeerK40t mirrors X against bed size).\[50\]
5. Only then test `A5 50 05` / `A5 51 05` on 50207 with the beam blocked.
6. Poll at 5 Hz during a LightBurn upload. A third-party daemon (laser-cutterd) reports this works and advises lowering the poll rate if uploads fail.\[52\] UNVERIFIED for Mod5.

Rule: read-only UDP until step 6 passes repeatedly.

### 6.6 Hardware I/O (VERIFIED: Ruida RDC6445G Control System manual V1.2)\[53\]

| Pin | Signal | Behavior |
|---|---|---|
| CN5-2 | L-ON1 | Glass-tube laser enable, "outputted in the low-level form"; unused for RF |
| CN5-3 | LPWM1 | Power PWM to glass PSU or RF PWM input |
| CN5-4 | WP1 | Low = normal; high = laser forced off, job suspended, alarm; **24 V logic only**\[53\]\[54\] |
| CN1-4 | Status | Opto-isolated OC, 500 mA; energized while working, released when work ends or stops\[53\] |
| CN2-2 | FootSW | Low ≥ 100 ms: start if idle, pause if working, resume if paused\[53\] |
| CN2-3 | DrProc | When enabled, high or floating = protected, work suspended, laser closed\[53\]\[55\] |
| CN2-4 | Shot | Low = manual laser on\[53\] |

Other inputs accept 5/12/24 V logic.\[53\]\[56\]

Wiring (firm):

- **Fire sense:** opto LED driven from L-ON1 (glass) or LPWM1 (RF); transistor to an ESP32 input with pullup. Fast opto if PWM edges matter (UNVERIFIED part).
- **Job running:** CN1 Status sinks an opto LED from +24 V.
- **Start:** ESP32 drives an opto that pulls FootSW low for 200 ms.
- **WP1 and DrProc:** emergency only, since both suspend and alarm.

### 6.7 Version 1 canned job (firm)

- One LightBurn job with three dots A, B, C. Use Line layers with Dot Mode and spacing longer than a very short line, so each line yields one dot; LightBurn staff describe this on the LightBurn forum, and Dot Mode is in the LightBurn docs (Line mode, Advanced).\[57\]\[58\]
- Loop: FootSW start, timestamp three fire events and integrate after each, wait for Status low, move (motors permitted only now), persist, repeat. The ESP32 holds the next start, so no in-job dwell is needed.
- Glass tubes typically do not lase reliably below about 8 to 15% (UNVERIFIED). Use the lowest power giving S ≥ 2·S_min, dot time 5 to 50 ms (UNVERIFIED). For RF, reject tickle (F5).

---

## 7. Safety logic

### 7.0 Pitfalls

- A one-shot on a sense line fails unsafe on a broken wire.
- MCU pins can glitch at reset; C6 strapping pins are sampled at reset.
- Coarse search fires an invisible 10.6 um beam into unplanned places.

### 7.1 Layered interlock (firm)

1. **Primary:** force-guided relay K1 driven by the coordinator ALIGN_MOVE output. NC contact in series with the laser PSU enable or water-protect loop; NO contact supplies MOTOR_PERMIT on the drag-chain inhibit line. Linked contacts mean permit implies laser loop open; any broken wire disables. PSU enable details are UNVERIFIED; check the PSU manual.
2. **Secondary:** retriggerable one-shot on fire sense forces inhibit for 100 ms.
3. **Tertiary:** node firmware checks inhibit, heartbeat and state.

Node gate: `nSLEEP = PERMIT_OK AND NOT(FIRE_RECENT) AND ARM_PULSE_OK`, pulled down by default. ARM_PULSE_OK is a diode-capacitor charge pump fed by a 1 kHz MCU square wave, so a stuck or resetting pin cannot enable. DRV8876 nSLEEP and EN have internal 100 kΩ pulldowns (VERIFIED); add 10 kΩ external on nSLEEP.\[20\]

### 7.2 One-shot timing

- 74LVC1G123: t_w ≈ K·R·C with K ≈ 1 for large C (UNVERIFIED; read the TI curve). 100 kΩ, 1 uF gives about 100 ms.
- RC plus Schmitt (CALCULATED): `t_hold = R*C*ln(1 / (1 - V_T+/V_DD))`; with V_T+/V_DD = 0.55, t_hold = 0.80·R·C, so 120 kΩ and 1 uF give 96 ms.
- Tickle rejection: low-pass before the trigger with a time constant of about 10 × tickle width (UNVERIFIED widths).

### 7.3 Coordinator state machine (firm)

| State | Motors | K1 laser loop | Enter | Exit |
|---|---|---|---|---|
| Inhibited | Off | Closed | Boot, cleared Fault, heartbeat lost | Idle after self-test |
| Idle | Off | Closed | Self-test OK | WaitingForFire |
| WaitingForFire | Off | Closed | FootSW pulsed | Measuring on fire edge; Fault after 30 s |
| Measuring | Off | Closed | Fire edge | WaitingForFire or Moving after Status low |
| Moving | On | **Open** | Status low, no fire for ≥ 500 ms, K1 feedback confirms open | Idle or Fault |
| Fault | Off | Closed | Stall, nFAULT, bus-off, 3 invalid shots, soft limit | Inhibited after manual clear |

Any fire edge in Moving is an immediate Fault; hardware has already dropped nSLEEP.

### 7.4 Boot pins

ESP32-C6 strapping pins: GPIO4, 5, 8, 9, 15; GPIO8 and GPIO9 select boot mode, GPIO9 has a weak pull-up (VERIFIED, ESP32-C6 datasheet v1.6).\[59\]\[60\]\[61\] Use them for nothing safety-related. Leave GPIO12/13 for USB-Serial-JTAG and GPIO16/17 for UART0 (UNVERIFIED secondary).\[60\]\[62\] ESP32-C3 boot mode is set by "Strapping pins: GPIO2, GPIO8, and GPIO9", with GPIO2 and GPIO8 floating by default and GPIO9 on a "Weak pull-up" (bit value 1) (VERIFIED, ESP32-C3 Series Datasheet v2.5). Per-pin boot glitch behavior is undocumented in sources consulted; the charge-pump ARM makes it irrelevant.

### 7.5 CO2 specifics

10.6 um is invisible. Keep the lid closed and lid interlock active during alignment shots. During coarse search the beam can hit the head body, sensor housing, drag chain and wiring, so cap power, duration and shot count, and check the sum signal before each further step. Never fire outside the known A, B, C coordinates during alignment.

---

## 8. Mechanical facts and prior art

### 8.0 Pitfalls

No published thread, pitch or dimensions for Cloudray K Series mounts were found. No N20 gearbox backlash data was found.

### 8.1 N20 / Pololu facts

| Item | Value | Status |
|---|---|---|
| Gearbox section, shaft | 10 × 12 mm; 3 mm D-shaft, 9 mm long | VERIFIED: Pololu #3069\[5\]\[63\] |
| Size, mass (298:1 extended shaft) | 10 × 12 × 26 mm, 9.5 g | VERIFIED as listed by RobotShop\[64\] |
| Encoder disc | 6 poles, 12 CPR, 7.65 mm OD on 1 mm shaft | VERIFIED: Pololu #2599\[65\] |
| Pololu encoder supply | 2.7 to 18 V | VERIFIED: Pololu #4761\[9\] |
| Generic 7 PPR N20 | 3.3/5 V, 14-pole ring, about 13 g | VERIFIED as vendor claims (NFP, Waveshare)\[10\]\[12\]\[66\] |

| Pololu 298:1 variant | No-load | Stall extrapolation |
|---|---|---|
| HPCB 6 V (#3069) | 110 RPM, 150 mA | 3.4 kg·cm, 1.5 A\[5\] |
| HP 6 V (#994) | 100 RPM, 100 mA | 4.0 kg·cm, 1.6 A\[67\] |
| LP 6 V (#1094) | 45 RPM, 40 mA | 2.0 kg·cm, 0.36 A\[68\] |
| HPCB 12 V (#3045) | 110 RPM | 3.3 kg·cm, 0.75 A\[5\] |

VERIFIED: Pololu pages. Pololu: "Stall torque and stall current specifications are theoretical values; stalls could damage the motor or gearbox."\[5\] Recommendation: HPCB 6 V 298:1 with integrated 12 CPR encoder (carbon brushes, about 11 um per count at 1 m).\[8\]\[69\]

### 8.2 Bambu Lab R1

| Item | Status |
|---|---|
| Bambu Lab R1, launched September 22, 2026 | VERIFIED: Bambu Lab press release on PR Newswire dated "AUSTIN, Texas, Sept. 22, 2026"; the Bambu Lab launch blog post ("Big Job. Light Work. Bambu Lab Launches R1") also states "This solution is exclusive to Bambu Lab." |
| "Auto Laser Alignment" with motorized mirror actuators and a removable Laser Alignment Sensor on the toolhead; the press release says it "corrects it with motorized mirror actuators. No paper, no test shots, no turning screws. In about one minute, it is done." (company claim) | VERIFIED: Bambu Lab press release (primary); also reported by All3DP |
| CO2 source | VERIFIED: Bambu Lab press release, "Bambu Lab today announced the launch of the R1, a 55W CO2 laser cutter and engraver" |
| Two gold brushed DC gearmotors under an amber cover (user photo) | INFERENCE: consistent with "motorized mirror actuators". The sensor is documented by Bambu Lab as "a plate with a crosshair - on the toolhead, and the machine reads the beam off the crosshair, finds where the path is off" (press release); motor type and algorithm are undocumented |
| H2D laser modules are 455 nm diode, camera placement | VERIFIED: Bambu Lab H2D pages;\[70\]\[71\] not mirror-alignment prior art |

A one-minute cycle implies a fast sensor, not seconds-per-shot thermal.

### 8.3 Benchmarks and prior art

- Newport Picomotor 8301NF: < 30 nm minimum incremental motion, 12.7 mm travel, 22 N load, 0.018 N·m drive torque, holds position unpowered; step size "varies from step to step and between forward and backward directions", so external position feedback is recommended (VERIFIED, Newport).\[72\]\[73\] The N20 chain at 0.14 um per count is coarser but adequate, and likewise relies on the sensor.
- Thorlabs and Zaber motorized mounts: not researched (evidence missing).
- DIY automatic CO2 alignment projects: none found. Near-field and far-field quadrant pairs for separating shift from tilt appear in beam-pointing literature (arXiv 1401.4981).\[74\]

---

## 9. To-measure checklist, constants, glossary

### 9.1 Checklist (in order)

1. Screw lead p (10 turns, calipers).
2. Lever arm L per motorized screw.
3. Which screw steers horizontal versus vertical (k).
4. Path lengths M1 to M2 at Y near/far; M2 to head at X near/far.
5. Usable screw range per axis (soft limits).
6. Running current at working duty (I_trip).
7. Minimum reliable move and creep duty; coast after brake.
8. Backlash per axis (3.4); settle time.
9. Sensor tau_cal, noise σ, S_ref; beam radius w at the head.
10. Minimum lasing power and dot time.
11. Ruida L-ON polarity, Status, FootSW behavior.
12. Ruida UDP test results (6.5).
13. Repeatability σ of a full A, B, C measurement.

### 9.2 Constants sketch

```rust
//! align_consts.rs: PLACEHOLDERS unless marked VERIFIED.
pub const GEAR_RATIO: f32 = 297.92; // VERIFIED Pololu #3069
pub const ENC_CPR: u32 = 12; // VERIFIED Pololu encoder
pub const MAX_INST_TORQUE_NM: f32 = 0.196; // VERIFIED Pololu guidance
pub const SCREW_LEAD_UM: f32 = 500.0; // UNVERIFIED
pub const LEVER_ARM_UM: [f32; 4] = [25_000.0; 4]; // UNVERIFIED
pub const DEFLECT_K: [f32; 4] = [2.0, 1.414, 2.0, 1.414]; // ASSUMED
pub const A_IPROPI_UA_PER_A: f32 = 1000.0; // VERIFIED DRV8876
pub const R_IPROPI_OHM: f32 = 6200.0; // CALCULATED
pub const I_TRIP_MA: f32 = 350.0; // CALCULATED
pub const VM_MV: u32 = 6000; // UVLO rising 4.45 V typ
pub const DUTY_MAX: f32 = 0.8;
pub const OVERSHOOT_COUNTS: i32 = 60; // set to 2x backlash
pub const STALL_WINDOW_MS: u32 = 50;
pub const SETTLE_MS: u32 = 200; // UNVERIFIED
pub const SOLVE_ALPHA: f32 = 0.6;
pub const CONVERGE_TOL_NORM: f32 = 0.02;
pub const COND_MAX: f32 = 50.0;
pub const HEARTBEAT_TIMEOUT_MS: u32 = 300;
pub const FIRE_HOLDOFF_MS: u32 = 100;
pub const CAN_BITRATE: u32 = 250_000; // UNVERIFIED choice
pub const RUIDA_PORT: u16 = 50200; // VERIFIED
pub const RUIDA_REPLY_PORT: u16 = 40200; // VERIFIED
pub const RUIDA_MAGIC: u8 = 0x88; // VERIFIED 644XG; Mod5 UNVERIFIED
pub const RUIDA_ACK_SWZ: u8 = 0xC6; // CALCULATED, matches jnweiger
pub const MEM_STATUS: [u8; 2] = [0x04, 0x00]; // VERIFIED MeerK40t
pub const MEM_X: [u8; 2] = [0x04, 0x21]; // VERIFIED MeerK40t
pub const MEM_Y: [u8; 2] = [0x04, 0x31]; // VERIFIED MeerK40t
pub const MEM_CARD_ID: [u8; 2] = [0x05, 0x7E]; // VERIFIED
```

### 9.3 Glossary

| Term | Meaning |
|---|---|
| A, B, C | The three measurement configurations (1.4) |
| DLS | Damped least squares, (JᵀJ + λ²I)⁻¹Jᵀ |
| Broyden | Rank-one Jacobian update from (Δq, Δm) |
| Standard direction | Screw advancing against springs; all final approaches end this way |
| IPROPI | DRV887x current-proportional output |
| PCNT | ESP32 hardware pulse counter with quadrature mode |
| Swizzle / magic | Ruida byte obfuscation and its per-model XOR constant |
| w | Gaussian 1/e² beam radius |

---

## 10. Caveats

- **Weakest evidence:** Mod5 protocol behavior, Cloudray mount data, N20 backlash, thermal sensor time constants, exact esp-hal 1.x TWAI and PCNT APIs, static-size nalgebra SVD in no_std, glass-tube minimum power, RF tickle widths. All are marked UNVERIFIED and listed in the checklist.
- **Third-party sources:** KerfDesk, laser-cutterd and some recent MeerK40t PRs appear partly AI-assisted and simulator-tested only; used as hints, not protocol facts.
- **Secondary counts:** the esp-hal 1.1 breaking-change details come from a secondary source; confirm against the crate changelogs for pinned versions. The C3 strapping pins (GPIO2, GPIO8, GPIO9, per the ESP32-C3 Series Datasheet v2.5) and the four PCNT units on the C6 and S3 (per the ESP32-C6 Series Datasheet v1.6 and ESP32-S3 Series Datasheet) are now confirmed by Espressif datasheets.
- **Bambu R1:** everything beyond "motorized mirror actuators plus removable toolhead sensor, about one minute" is inference from the user's photo.

## Sources

1. [GitHub - madhephaestus/ESP32Encoder: A Quadrature and half quadrature PCNT peripheral driven encoder library supporting 8 encoders · GitHub](https://github.com/madhephaestus/ESP32Encoder)
2. [RotaryEncoderPCNT - Arduino Libraries](https://www.arduinolibraries.info/libraries/rotary-encoder-pcnt)
3. [github.com](https://github.com/espressif/arduino-esp32/issues/6489)
4. [Ruida - EduTech Wiki](https://edutechwiki.unige.ch/en/Ruida)
5. [Pololu - 298:1 Micro Metal Gearmotor HPCB 6V](https://www.pololu.com/product/3069)
6. [esp\_hal::pcnt - Rust](https://docs.espressif.com/projects/rust/esp-hal/1.0.0-beta.0/esp32c6/esp_hal/pcnt/index.html)
7. [esp-hal — embedded dev in Rust // Lib.rs](https://lib.rs/crates/esp-hal)
8. [Pololu - 15:1 Micro Metal Gearmotor HP 6V with 12 CPR Encoder, Side Connector](https://www.pololu.com/product/5157)
9. [Pololu - Magnetic Encoder Pair Kit with Side-Entry Connector for Micro Metal Gearmotors, 12 CPR, 2.7-18V](https://www.pololu.com/product/4761)
10. [Waveshare N20 DC Gear Motor Magnetic Hall Encoder All-metal Gearbox L-shaped 6P](https://www.ebay.com/itm/256419614282)
11. [N20 dc motor uses a magnetic disc and Hall effect sensors to provide 12 counts per revolution of the motor shaft. The sensors operate from 3.3 V to 24 V and provide digital outputs that can be connected directly to a microcontroller or other digital circuit.](https://kegumotor.com/en/product/n20-gear-motor-encoder.html)
12. [n20 dc gear motor with encoder: Reliable Speed Control Solutions](https://microdcmotors.com/product/micro-dc-gear-motor-reduction-motor-gearbox-model-nfp-gm12-n20-00301-10d)
13. [High Precision Position Measurement Method for Laguerre-Gaussian Beams Using a Quadrant Detector - PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC6263427/)
14. [High Precision Position Measurement Method for Laguerre-Gaussian Beams Using a Quadrant Detector](https://doi.org/10.3390/s18114007)
15. [A novel method to improve detecting sensitivity of quadrant detector - ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0030402614002150)
16. [(PDF) Improved measurement accuracy of the quadrant detector through improvement of linearity index](https://www.researchgate.net/publication/234941266_Improved_measurement_accuracy_of_the_quadrant_detector_through_improvement_of_linearity_index)
17. [WebAssembly and embedded targets](https://www.nalgebra.org/docs/user_guide/wasm_and_embedded_targets/)
18. [numeris — Rust math library // Lib.rs](https://lib.rs/crates/numeris)
19. [Pulse Counter (PCNT) Module - circuitlabs.net](https://circuitlabs.net/pulse-counter-pcnt-module/)
20. [DRV887x H-Bridge Motor Driver 4.5 to 37 V Controller Control Inputs nSLEEP](https://www.ti.com/lit/ds/symlink/drv8876.pdf)
21. [Support ISR based pulse counter on ESP32-C3 by agners · Pull Request #2983 · esphome/esphome · GitHub](https://github.com/esphome/esphome/pull/2983)
22. [ESP32-C6 Series - Espressif Documentation](https://documentation.espressif.com/esp32-c6_datasheet_en.pdf)
23. [ESP32 dual twai / Canbus - General Guidance - Arduino Forum](https://forum.arduino.cc/t/esp32-dual-twai-canbus/1426071)
24. [ESP32 SoC Table](https://tanakamasayuki.github.io/ESP-IDF-SoC-Check/soc.html)
25. [fix(firmware): full quadrature encoder decoding on PCNT via ESP32Encoder (#69) by michmos · Pull Request #72 · michmos/mobile\_robot · GitHub](https://github.com/michmos/mobile_robot/pull/72)
26. [DRV887x H-Bridge Motor Driver 4.5 to 37 V Controller Control Inputs nSLEEP](http://www.visvie.com/static/upload/file/20250527/1748330577521606.pdf)
27. [DRV887x H-Bridge Motor Driver 4.5 to 37 V Controller Control Inputs nSLEEP](https://www.ti.com/lit/ds/symlink/drv8874.pdf)
28. [DRV8876-Q1 H-Bridge Motor Driver With Integrated Current Sense and Regulation](https://www.ti.com/lit/ds/symlink/drv8876-q1.pdf)
29. [DRV8876-Q1 H-Bridge Motor Driver With Integrated Current Sense and Regulation](https://www.visvie.com/static/upload/file/20250528/1748402921231856.pdf)
30. [DRV8833 www.ti.com SLVSAR1C –JANUARY 2011–REVISED JANUARY 2013](https://users.ece.utexas.edu/~valvano/Datasheets/drv8833.pdf)
31. [DRV8833 www.ti.com SLVSAR1B –JANUARY 2011–REVISED AUGUST 2011](https://datasheet.octopart.com/DRV8833PW-Texas-Instruments-datasheet-10383660.pdf)
32. [PWM nSLEEP nFAULT 2.7 to 10.8 V Controller M + t + t DRV8833](https://cdn-learn.adafruit.com/assets/assets/000/039/902/original/drv8833.pdf?1488941266=)
33. [Pololu Micro Metal Gearmotor (6V)](https://shop.pimoroni.com/en-us/products/pololu-micro-metal-gearmotor-6v)
34. [ADC: feature completeness · Issue #6747 · esp-rs/esp-hal · GitHub](https://github.com/esp-rs/esp-hal/issues/6747)
35. [Releases · esp-rs/esp-hal · GitHub](https://github.com/esp-rs/esp-hal/releases)
36. [esp-hal-embassy — embedded dev in Rust // Lib.rs](https://lib.rs/crates/esp-hal-embassy)
37. [fix: update dependencies by sam-ruff · Pull Request #1 · sam-ruff/walkie-textie-firmware · GitHub](https://github.com/sam-ruff/walkie-textie-firmware/pull/1)
38. [esp-rtos - crates.io: Rust Package Registry](https://crates.io/crates/esp-rtos)
39. [TWAI: feature completeness · Issue #6781 · esp-rs/esp-hal](https://github.com/esp-rs/esp-hal/issues/6781)
40. [Embassy Book](https://embassy.dev/book/)
41. [Async - The Rust on ESP Book](https://docs.espressif.com/projects/rust/book/application-development/async.html)
42. [GitHub - infinition/waveshare-watch-rs: 100% Rust \`no\_std\` smartwatch firmware for the Waveshare ESP32-S3-Touch-AMOLED-2.06 · GitHub](https://github.com/infinition/waveshare-watch-rs)
43. [Two-Wire Automotive Interface (TWAI) - ESP32-C6 - — ESP-IDF Programming Guide v5.4.3 documentation](https://docs.espressif.com/projects/esp-idf/en/v5.4.3/esp32c6/api-reference/peripherals/twai.html)
44. [Two-Wire Automotive Interface (TWAI) - ESP32-C6 - — ESP-IDF Programming Guide release-v5.1 documentation](https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32c6/api-reference/peripherals/twai.html)
45. [Two-Wire Automotive Interface (TWAI) - ESP32-C6 - — ESP-IDF Programming Guide v6.1 documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/twai.html)
46. [Using Meerk40t to help create Ruida laser management software? - Meerk40t - Maker Forums](https://forum.makerforums.info/t/using-meerk40t-to-help-create-ruida-laser-management-software/84880)
47. [meerk40t/meerk40t/ruida/rdjob.py at main · meerk40t/meerk40t](https://github.com/meerk40t/meerk40t/blob/main/meerk40t/ruida/rdjob.py)
48. [ruida-laser/doc/protocol.md at master · jnweiger/ruida-laser](https://github.com/jnweiger/ruida-laser/blob/master/doc/protocol.md)
49. [fix(controller): audit follow-up — console exchanges, alarm scope, recovery restart points, desktop connect (ADR-362) by cisgz3a-hub · Pull Request #871 · cisgz3a-hub/KerfDesk · GitHub](https://github.com/cisgz3a-hub/KerfDesk/pull/871)
50. [meerk40t/meerk40t/ruida/controller.py at main · meerk40t/meerk40t](https://github.com/meerk40t/meerk40t/blob/main/meerk40t/ruida/controller.py)
51. [Live Ruida control over UDP on the desktop (ADR-697) by cisgz3a-hub · Pull Request #1707 · cisgz3a-hub/KerfDesk · GitHub](https://github.com/cisgz3a-hub/KerfDesk/pull/1707)
52. [GitHub - romkey/laser-cutterd: Monitors a Ruida-controlled laser cutter over the network and reports its status over MQTT, with Home Assistant discovery. · GitHub](https://github.com/romkey/laser-cutterd)
53. [RDC6445G Control System manual V1.2 SHENZHEN RUIDA TECHNOLOGY RDC6445G](https://www.ruidacontroller.com/wp-content/uploads/2021/10/RDC6445G-Control-System-V1.2-Manual.pdf)
54. [RDC6445G/S Control System manual SHENZHEN RUIDA TECHNOLOGY RDC6445G/S](https://www.ruidacontroller.com/wp-content/uploads/2021/10/Users-Manual-of-RDC6445G-S-Control-System-V1.5.pdf)
55. [User’s Manual of RDC6445G Control System Shenzhen Reader Technology Co., Ltd.](https://cloudray.oss-us-west-1.aliyuncs.com/Controller/Ruida/RDC6445G/User's%20Manual%20of%20RDC6445G%20V1.0.pdf)
56. [User’s Manual of RDC6445GZ Control System SHENZHEN RUIDA TECHNOLOGY RDC6445GZ](https://www.ruidacontroller.com/wp-content/uploads/2021/10/RDC6445GZ-Laser-Cutting-System-Hardware-User-Manual-V1.3.pdf)
57. [Beam pulsing via software - Community Laser Talk - LightBurn Software Forum](https://forum.lightburnsoftware.com/t/beam-pulsing-via-software/2367)
58. [Line Mode - LightBurn Documentation](https://docs.lightburnsoftware.com/UI/CutSettings/CutSettings-Line.html)
59. [ESP32-C6 Series Datasheet Version 1.6](https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf)
60. [ESP32-C6 DevKitC-1 Pinout & Safe GPIO Guide](https://esp32.co.uk/esp32-c6-devkitc-1-pinout-diagram-safe-gpios-v1-2/)
61. [ESP32-C6 Pinout: GPIO, ADC, Strapping Pins & LP Core](https://solderhub.com/articles/esp32-c6-pinout-guide)
62. [ESP32-C6-WROOM-1 Pinout](https://www.studiopieters.nl/esp32-c6-wroom-1-pinout/)
63. [Pololu - 10:1 Micro Metal Gearmotor MP 6V with 12 CPR Encoder, Back Connector](https://www.pololu.com/product/5128)
64. [6V Pololu 298:1 Micro Metal Gearmotor w/ Extended Motor Shaft - RobotShop](https://www.robotshop.com/products/6v-pololu-2981-micro-metal-gearmotor-w-extended-motor-shaft)
65. [Pololu - Magnetic Encoder Disc for Micro Metal Gearmotors, OD 7.65 mm, ID 1.0 mm, 12 CPR (Bulk)](https://www.pololu.com/product/2599)
66. [n20 Small DC Gear Motor with Encoder: Versatile and Durable](https://nfpshop.com/product/12mm-dc-gearmotor-24mm-type-model-nfp-jga12-n20-with-encoder)
67. [Pololu - 298:1 Micro Metal Gearmotor HP 6V](https://www.pololu.com/product/994)
68. [Pololu - 298:1 Micro Metal Gearmotor LP 6V](https://www.pololu.com/product/1094)
69. [Pololu - Micro Metal Gearmotors](https://www.pololu.com/category/60/micro-metal-gearmotors)
70. [Buy Bambu Lab H2D: Dual-Nozzle 3D Printer + Laser All-in-One](https://us.store.bambulab.com/products/h2d?id=622679274382356488)
71. [Bambu Lab H2D 3D Printer](https://ultimate3dprintingstore.com/products/bambu-lab-h2d-3d-printer)
72. [8301NF Picomotor Piezo Linear Actuator](https://www.newport.com.cn/p/8301NF)
73. [8301-UHV-KAP Picomotor Actuator](https://www.newport.com/p/8301-UHV-KAP)
74. [A sub nrad beam pointing monitoring and stabilization system for controlling input beam jitter in GW interferometers](https://arxiv.org/pdf/1401.4981)
