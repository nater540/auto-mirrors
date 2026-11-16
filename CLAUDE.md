# auto-mirrors

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

A closed-loop system that aligns the two steering mirrors of a flying-optics CO2 laser without manual screw turning.
Four 28BYJ-48 steppers turn adjuster screws (two per mirror), a quadrant beam sensor at the head reports beam
position over CAN, and an ESP32-C6 coordinator computes corrections with an empirical Jacobian and damped least
squares. The Ruida RDC6445G keeps full control of gantry motion and laser firing: this system never fires the laser.

Hardware is still being designed. The work so far is host-testable logic. A bare ESP32-C6 development board (no
motors, relay, or bus attached) runs the M0 scaffold images; the owner allowed Claude to flash it on 2026-11-15.

## Read first

| File | What it is |
|---|---|
| `docs/HANDOFF.md` | The owner's hand-off: settled decisions, safety rules, working agreement, milestones, open items |
| `docs/ADDENDUM_A_28BYJ48.md` | Stepper actuator, Series E mount geometry, constants |
| `docs/REFERENCE.md` | Optics and sensor math, solver, esp-hal notes, Ruida protocol and I/O, safety logic |
| `docs/VERSIONS.md` | Pinned crate versions, why, and what contradicted the reference |

**Precedence when they disagree: the hand-off, then the addendum, then the reference.** The hand-off also wins over
the house style below.

Facts in the documents carry evidence tags: VERIFIED, CALCULATED, UNVERIFIED, AS MODELED (and ASSUMED, FIRM).
**Never promote a value that is not VERIFIED or CALCULATED to a literal.** It goes in `align_core::config` with its
tag in the doc comment. Do not replace a measured-later value with an invented one: leave the tagged placeholder and
add it to the open list in the report.

## Milestone status

Stop at the end of each milestone and report to the owner (hand-off section 6).

- M0 scaffold: done. M1 core math: done. M2 simulator and closed loop: done. M3 protocol: done.
- M4 stepper and node firmware: next (needs the pin assignments, hand-off open item 7). M5 coordinator firmware,
  M6 bring-up tools, M7 Ruida UDP (optional, read-only): not started.

## Things that must not be done

- Do not send any Ruida command that moves an axis or fires the laser.
- Do not add a code path that energizes motors at boot, or without a received command and a present permit.
- Do not flash hardware, or ask the owner to flash, without stating what the image does to motors and I/O. Flash
  only through `just flash`, and only an image whose notice was read in the same session.
- Never write code that assumes firmware alone prevents motion during firing. Firmware checks are the last layer,
  behind the force-guided relay and the switched motor rail.
- Never use strapping pins GPIO4, 5, 8, 9, 15 for a safety signal. Every pin lives in one `board.rs` per firmware
  crate (none exists yet: there are no pin assignments).
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

- **A step is judged only if its predicted change is above the noise floor** (`calibration::noise_floor`, five
  sigmas of a measurement component). Addendum A.5 as written reports a healthy axis as not responding, because
  corrections of a few steps are inside stick-slip and noise.
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
- The absolute two-position target is kept out of the product and exists only in
  `crates/align-sim/tests/closed_loop.rs`, through the `Aligner::with_target` seam.

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
- **Not yet defined, carried as raw bytes:** stop and estop reasons, the inhibit source, the heartbeat state, and the
  sensor kind. Give them meaning in the milestone that uses them.
- **Duplicate moves (`MoveRelTracker`, one per node):** the last sequence number with identical content is answered
  (`AlreadyRunning`, or the same MOVE_DONE again) and never run twice; with different content it is refused
  (`SeqReused`); a new move while one runs is refused (`Busy`). The tracker is RAM only: after a reboot it accepts
  anything, so the node must restore its position before answering a move.

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
  is an UNVERIFIED placeholder and a scenario parameter, never configuration.
- `crates/stepper`: `no_std`. 28BYJ-48 driver over `embedded_hal::digital::OutputPin`. Empty until M4.
- `firmware/node`, `firmware/coordinator`: ESP32-C6 binaries (`align-node`, `align-coordinator`). Scaffold only:
  they boot, log one line, and touch no pin.

Dependency direction: firmware and `align-sim` depend on `align-core`, `align-proto`, and `stepper`; `align-core`
depends on none of them. HAL types never enter the `crates/` libraries.

The firmware crates are workspace members but not `default-members`: a plain `cargo build`, `cargo test`, or
`cargo clippy` at the root works on the host crates only. **`--workspace` tries to build esp-hal for the host and
fails.** Build firmware by package and target, through the justfile.

## Commands

`just` lists every recipe by group. The ones used most:

```bash
just ci                    # everything that must pass before a commit, cheapest first
just test                  # host tests; `just test align-core` for one crate
just clippy                # host crates and tests, warnings denied (`just lint` is the same)
just fmt                   # format everything
just doc                   # API docs with warnings denied
just check-no-std          # build the no_std libraries for the ESP32-C6 (proves no std, no alloc)
just firmware              # build both images in release mode; never flashes
just image node            # write the flashable image to target/images/ and print its size; touches no board
just ports                 # serial ports, the Espressif ones apart from the rest
just flash node            # print the notice, need it acknowledged, flash, then monitor or check the boot log
just boot-log coordinator  # reset the board and check that its boot log holds the coordinator's whole notice
just sim --seed 7          # one simulated alignment, every move and measurement
just sweep --runs 5000     # convergence against conditioning (the Y span)
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

**The board recipes talk to hardware: `flash`, `boot-log`, `monitor`, `reset`, `board-info`, `ports`.** Their
logic is in `scripts/board.sh`.

- `just flash node` (or `coordinator`) is the only way to flash. It prints the crate's `FLASH_NOTICE.txt` and needs
  it acknowledged: the crate name typed at a terminal, or `ALIGN_FLASH_ACK=<crate>:<hash>` without one. The hash is
  the start of the notice's SHA-256 and the recipe prints it after the notice, so an acknowledgement goes stale
  the moment the notice changes. Claude flashes this way, and only after reading the notice it acknowledges, and
  says in its reply what the image does to motors and I/O. There is no cargo runner, on purpose: `cargo run` must
  not flash around the notice.
- Without a terminal, `flash` ends by reading the boot log and failing unless every line of the notice appears
  and nothing panicked: that proves which image is running. `just boot-log <crate>` does the same check without
  flashing. The logs land in `target/boot-logs/`.
- With no port named, the board recipes use the only Espressif USB port (vendor 303A) and stop when there are none
  or several. The LG monitor on this machine also shows up as two `usbmodem` ports: never pick a port by name
  pattern.
- **`firmware/<crate>/FLASH_NOTICE.txt` states what the image does to motors and I/O.** The image logs it at boot.
  Change it in the same commit as any change to what the firmware drives; a stale notice is a safety defect.
- Do not run `monitor`, `reset`, or `board-info` on a port without knowing what is on it. Other boards are often
  attached to this machine.

## Code style: non-negotiable

- **Two-space indentation, exclusively.** Rust, TOML, YAML, Markdown, shell: everything except the `justfile`
  (four spaces). Never let rustfmt or an editor reformat to four.
- **Aim for ~120-character lines**, comments and Markdown included. Break where the text has a natural pause; do
  not pad lines out to the limit.
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
