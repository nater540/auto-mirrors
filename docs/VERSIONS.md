# Pinned versions

Checked on 2026-11-15 against crates.io and the crate sources under `~/.cargo/registry`. Every claim in the
"Checked how" column was run or read on that day; nothing here comes from memory or from the snippets in
`REFERENCE.md`.

The esp-hal family changes API between minor releases. The crates below are pinned with `=` in the root
`Cargo.toml` and must move together. `Cargo.lock` is committed and pins everything else.

## Toolchain

| Item | Pin | Why | Checked how |
|---|---|---|---|
| Rust | 1.96.0 (`rust-toolchain.toml`) | The floor is 1.95.0, set by esp-hal 1.2.2 and esp-rtos 0.4.0. 1.96.0 is the version the owner's other projects pin and the one installed here. Stable is 1.100.0; nothing needs it yet. | `cargo info`, `rustup check` |
| Target | `riscv32imac-unknown-none-elf` | ESP32-C6. A prebuilt target on stable: no espup, no Xtensa fork, no `build-std`. | Both firmware crates build with the pinned toolchain |
| esp-generate | 2.0.0 | Source of the firmware skeleton. The copy installed on this machine was 1.3.0, which generates the previous generation (esp-hal ~1.1.0, esp-rtos 0.3.0). 2.0.0 was installed into a scratch directory for this run; the global install was left alone. | `esp-generate --help` of both versions |

## Firmware crates (exact pins)

| Crate | Pin | Features | Why this version |
|---|---|---|---|
| esp-hal | =1.2.2 | `esp32c6`, `log-04`, `unstable` | Current release, and what esp-generate 2.0.0 emits. `unstable` is required for TWAI and ADC. |
| esp-rtos | =0.4.0 | `embassy`, `esp32c6`, `log-04` | Current release. Supplies the Embassy executor and time driver (it replaced esp-hal-embassy). |
| esp-bootloader-esp-idf | =0.6.0 | `esp32c6`, `log-04` | Supplies the app descriptor the second-stage bootloader requires. |
| esp-backtrace | =0.20.0 | `esp32c6`, `panic-handler`, `println` | Panic handler. |
| esp-println | =0.18.0 | `esp32c6`, `log-04` | `log` back end over USB-Serial-JTAG. |
| embassy-executor | =0.10.0 | `log` | Current release; the version esp-rtos 0.4.0 is built against. |
| embassy-time | =0.5.1 | `log` | Current release (the template asks for 0.5.0; 0.5.1 is a patch). |

Logging is `log` with esp-println, not defmt. Reason: milestones M4 to M6 need a console a person can read and
paste back, and defmt output needs a host decoder. This is one choice in two `Cargo.toml` feature lists and can
change.

## Host and shared crates

| Crate | Pin | Features | Why |
|---|---|---|---|
| nalgebra | =0.35.0 | `libm`, no default features | Fixed-size `Matrix4<f32>`, Cholesky, SVD. Minimum Rust 1.89. |
| libm | 0.2.16 | | Float functions `core` lacks (rounding). |
| thiserror | 2.0.21 | no default features | Fault enums; works without `std`. |
| embedded-can | 0.4.1 | | `align-proto`: the `Frame` trait and `StandardId`, which the esp-hal 1.2.2 TWAI frame implements (`EspTwaiFrame`), so firmware frames go straight into the decoder. Released 2022-09-28, past the one-year dormancy rule; kept because it is a stable trait crate in the active rust-embedded/embedded-hal repository and esp-hal already depends on it. Checked 2026-11-15. |
| bitflags | 2.13.2 | no default features | `align-proto`: the STATUS flag byte. Released 2026-09-10; already in the tree. Checked 2026-11-15. |
| proptest | 1.12 | `std`, no default features | Property tests, host only. |
| rand | 0.10.4 | | Simulator only: seeded sensor noise and skipped steps. In 0.10 the convenience methods are on `RngExt`, not `Rng`. |
| rand_distr | 0.6.0 | | Simulator only: the standard normal distribution. |
| clap | 4.6 | `derive` | Simulator command line. |
| anyhow | 1.0.104 | | Simulator command line, at the top level of the binary only. |

## Checked, not yet a dependency

These were looked up so that later milestones start from current facts. Check them again when they are added.

| Crate | Current | Needed for |
|---|---|---|
| embassy-sync | 0.8.0 | M4: channels, `Signal`, `Watch` (all three modules exist in 0.8.0) |
| embedded-hal | 1.0.0 | M4: `OutputPin` for the stepper driver |
| esp-storage | 0.10.0 | M4: flash persistence |
| sequential-storage | 8.0.2 | M4: persistence map (the reference names it; `ekv` 1.0.0 is the alternative) |

## What contradicted the reference

1. **`esp_rtos::start` signature.** Reference 5.0 says esp-rtos 0.3 takes a `SoftwareInterrupt<'static, 0>` plus
   the timer. In 0.4.0 it is `start(timer: impl TimerSource, int0: FROM_CPU_INTR0<'static>)`: the timer first, then
   the `FROM_CPU_INTR0` peripheral singleton (esp-rtos changelog, 0.4.0, change #6142). Read from the pinned
   source and confirmed by the build.
2. **nalgebra fixed-size SVD without alloc** was UNVERIFIED in reference 3.3. It is now confirmed for 0.35.0:
   `SVD::try_new` and `cholesky` on `Matrix4<f32>` build for `riscv32imac-unknown-none-elf` with
   `default-features = false, features = ["libm"]`. No hand-written Jacobi SVD is needed. `just check-no-std`
   keeps proving it.
3. **The reference solver sketch can panic.** `j.svd(false, false)` has no iteration cap, `dq[i]` indexes, and
   `f32::clamp` panics when a limit is negative or NaN. `align-core` uses `SVD::try_new` with an iteration cap
   and returns faults as values instead.
4. **The esp-generate template sets `[unstable] build-std = ["core"]`** next to a stable toolchain. It is left
   out here: the target is prebuilt and both images build without it.
5. **esp-generate 2.0.0 changed its command line.** The chip is now a template option (`-o esp32c6`), no longer
   `--chip esp32c6`. The command used was
   `esp-generate --headless -o esp32c6 -o unstable-hal -o embassy -o log -o esp-backtrace`. No module option was
   selected, because the module on the boards is not decided.

Confirmed as the reference states: esp-hal-embassy and esp-wifi are gone (esp-rtos and esp-radio replace them);
esp-rtos 0.4.0 is current with minimum Rust 1.95; TWAI sits behind the `unstable` feature in esp-hal 1.2.2, offers
`BaudRate::B250K`, and its `transmit` retries until the controller goes error-passive, so every transmit still
needs a timeout.

## Upgrade procedure

1. Read the changelog of every esp crate between the pin and the target version. Move all of them in one change.
2. Generate a fresh project with the matching esp-generate and compare its `Cargo.toml` and `main.rs` with
   `firmware/`.
3. `just ci`.
4. Update the tables above, including the date.
