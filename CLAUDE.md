# auto-mirrors

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

A closed-loop system that aligns the two steering mirrors of a flying-optics CO2 laser without manual screw turning.
Four 28BYJ-48 steppers turn adjuster screws (two per mirror), a quadrant beam sensor at the head reports beam
position over CAN, and an ESP32-C6 coordinator computes corrections with an empirical Jacobian and damped least
squares. The Ruida RDC6445G keeps full control of gantry motion and laser firing: this system never fires the laser.

Hardware is still being designed. The work so far is host-testable logic; nothing has been flashed.

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

- M0 scaffold: done. M1 core math: done.
- M2 simulator and closed loop: next. M3 protocol, M4 stepper and node firmware, M5 coordinator firmware, M6
  bring-up tools, M7 Ruida UDP (optional, read-only): not started.

## Things that must not be done

- Do not send any Ruida command that moves an axis or fires the laser.
- Do not add a code path that energizes motors at boot, or without a received command and a present permit.
- Do not flash hardware, or ask the owner to flash, without stating what the image does to motors and I/O.
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
- Three adopted recommendations await the owner's sign-off (ESP32-C6, the three-position measurement, the
  force-guided relay as primary interlock). Build to them.

## Workspace layout

One Cargo workspace, edition 2024, Rust 1.96.0 pinned. Dependencies and lints are declared once in the root
`Cargo.toml`.

- `crates/align-core`: `no_std`, no HAL. Config, unit conversions, quadrant math, the measurement vector, the solver.
  Later: Jacobian calibration, the approach planner, the coordinator state machine (a pure function of state and
  event), and the `Actuator` and `BeamSensor` traits.
- `crates/align-proto`: `no_std`. CAN identifiers and message encoding. Empty until M3.
- `crates/align-sim`: `std`. Optics and mount simulator, closed-loop tests, CLI runner. Empty until M2.
- `crates/stepper`: `no_std`. 28BYJ-48 driver over `embedded_hal::digital::OutputPin`. Empty until M4.
- `firmware/node`, `firmware/coordinator`: ESP32-C6 binaries (`align-node`, `align-coordinator`). Scaffold only:
  they boot, log one line, and touch no pin.

Dependency direction: firmware and `align-sim` depend on `align-core`, `align-proto`, and `stepper`; `align-core`
depends on none of them. HAL types never enter the `crates/` libraries.

The firmware crates are workspace members but not `default-members`: a plain `cargo build`, `cargo test`, or
`cargo clippy` at the root works on the host crates only. **`--workspace` tries to build esp-hal for the host and
fails.** Build firmware by package and target, through the justfile.

## Commands

```bash
just               # list the recipes
just ci            # everything that must pass before a commit
just test          # host tests; `just test align-core` for one crate
just clippy        # host crates and tests, warnings denied
just fmt           # format everything
just check-no-std  # build the no_std libraries for the ESP32-C6 (proves no std, no alloc)
just firmware      # build both images in release mode; never flashes
just firmware-clippy
cargo test -p align-core <test_name>                     # one test
```

There is no flashing recipe and no cargo runner yet, on purpose. `cargo deny` is not configured yet.

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
- Prefer a well-maintained crate to hand-written infrastructure, and avoid crates that have been dormant for over a year. Do not add one for something that is a few correct lines.
- Turn off default features and enable only what is used; put a comment above a dependency saying why it is there when that is not obvious.
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
