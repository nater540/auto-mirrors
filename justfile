# Project actions. `just` with no arguments lists them. https://just.systems
#
# Nothing here flashes a board. A flashing recipe, when one is added, must say what the image does to motors and
# I/O before it runs (docs/HANDOFF.md, section 7).

set shell := ["bash", "-euo", "pipefail", "-c"]

# The ESP32-C6 target, and the crates that are built for it.
target := "riscv32imac-unknown-none-elf"
firmware_crates := "-p align-node -p align-coordinator"
no_std_crates := "-p align-core -p align-proto -p stepper"

default:
    @just --list --unsorted

# --- host -------------------------------------------------------------------

# Debug build of the host crates
build:
    cargo build

# Run every host test, or one crate's: `just test align-core`
test crate="":
    {{ if crate == "" { "cargo test" } else { "cargo test -p " + crate } }}

# Clippy on the host crates and their tests, warnings denied
clippy:
    cargo clippy --all-targets -- -D warnings

# Format every crate, firmware included
fmt:
    cargo fmt --all

# Fail if any file is not formatted
fmt-check:
    cargo fmt --all --check

# --- ESP32-C6 ---------------------------------------------------------------

# Build the no_std library crates for the ESP32-C6: proves they need neither std nor alloc
check-no-std:
    cargo build {{ no_std_crates }} --target {{ target }}

# Build both firmware images in release mode (build only, never flashes)
firmware:
    cargo build {{ firmware_crates }} --target {{ target }} --release

# Clippy on the firmware crates, warnings denied
firmware-clippy:
    cargo clippy {{ firmware_crates }} --target {{ target }} --release -- -D warnings

# --- everything -------------------------------------------------------------

# Everything that must pass before a commit: format, lints, host tests, and the ESP32-C6 builds
ci: fmt-check clippy test check-no-std firmware-clippy firmware
