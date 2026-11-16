# Project actions. `just` with no arguments lists them by group. https://just.systems
#
# Two rules shape this file:
#
# - Plain cargo commands at the repository root see only the host crates (the workspace `default-members`). The
#   firmware crates build for the ESP32-C6 only, so every firmware recipe names its packages and the target.
# - Nothing reaches a board unless what it does was stated and acknowledged (docs/HANDOFF.md, section 7). `flash`
#   prints the crate's FLASH_NOTICE.txt and needs either the crate name typed at a terminal or
#   ALIGN_FLASH_ACK=<crate>:<hash of that notice>, so an acknowledgement goes stale when the notice changes. There
#   is no cargo runner, so `cargo run` cannot flash around it.

set shell := ["bash", "-euo", "pipefail", "-c"]

# The ESP32-C6 target, and the crates that are built for it.
target := "riscv32imac-unknown-none-elf"
chip := "esp32c6"
firmware_crates := "-p align-node -p align-coordinator"
no_std_crates := "-p align-core -p align-proto -p stepper -p node-logic"
release_dir := "target" / target / "release"

# The board actions: port selection, the flash gate, flashing, and the boot log check.
board := "scripts/board.sh"

# The update check is a network call on every espflash run, and its banner clutters captured boot logs.
export ESPFLASH_SKIP_UPDATE_CHECK := "true"

# The simulator binary, built by `_sim`.
sim_bin := "target/release/align-sim"

# For a mutation run: one fixed proptest seed, so that a mutant is caught or missed the same way every time, and no
# regression files written into the copied tree.
mutants_env := "PROPTEST_RNG_SEED=1 PROPTEST_DISABLE_FAILURE_PERSISTENCE=1"

alias lint := clippy

[private]
default:
    @just --list --unsorted

# --- host -------------------------------------------------------------------

# Type-check the host crates and their tests, without building
[group('host')]
check:
    cargo check --all-targets

# Debug build of the host crates
[group('host')]
build:
    cargo build

# Run every host test, or one crate's: `just test align-core`
[group('host')]
test crate="":
    {{ if crate == "" { "cargo test" } else { "cargo test -p " + crate } }}

# The property tests at many more cases than the default 256: `just test-stress 5000`
[group('host')]
test-stress cases="2000":
    PROPTEST_CASES={{ cases }} cargo test

# Clippy on the host crates and their tests, warnings denied
[group('host')]
clippy:
    cargo clippy --all-targets -- -D warnings

# Format every crate, firmware included. Nightly rustfmt: rustfmt.toml uses `trailing_comma`, an unstable option
[group('host')]
fmt:
    cargo +nightly fmt --all

# Fail if any file is not formatted (nightly rustfmt, as `fmt`)
[group('host')]
fmt-check:
    cargo +nightly fmt --all --check

# Build the API docs of the host crates, warnings (a broken link, a missing doc) denied
[group('host')]
doc:
    RUSTDOCFLAGS="-D warnings" cargo doc --no-deps

# Build the API docs and open them in a browser
[group('host')]
doc-open:
    cargo doc --no-deps --open

# --- simulator --------------------------------------------------------------

# One simulated alignment with every move and measurement: `just sim --seed 7 --stick 5`, `just sim --help`
[group('simulator')]
sim *args: _sim
    {{ sim_bin }} run {{ args }}

# Convergence against conditioning (the Y span): `just sweep --runs 5000`, `just sweep --help`
[group('simulator')]
sweep *args: _sim
    {{ sim_bin }} sweep {{ args }}

# How much stick-slip, noise, backlash, and step loss the starting thresholds tolerate, at the 500 mm span
[group('simulator')]
sweep-margins runs="5000": _sim
    #!/usr/bin/env bash
    set -euo pipefail
    # One row of a sweep at the 500 mm span, with its label in front and its fault tally under it.
    row() {
      local label="$1"
      shift
      {{ sim_bin }} sweep --runs {{ runs }} --y-spans 500 "$@" | tail -n +3 \
        | sed -e "1s/^/$(printf '%-22s' "$label")/" -e "2,\$s/^/$(printf '%-22s' "")/"
    }
    echo "{{ runs }} runs per row, default config, placeholder machine (UNVERIFIED numbers)"
    printf '%-22s%s\n' "" "y_span_mm  model_cond  converged  first_attempt  mean_steps  recalibrated  faults"
    echo "seat stick-slip, steps: calibrating first"
    for steps in 0 1 2 3 5 8; do row "  stick $steps" --stick "$steps"; done
    echo "seat stick-slip, steps: from a stored Jacobian 30% wrong"
    for steps in 0 1 2 3 5 8; do row "  stick $steps" --stick "$steps" --jacobian-error 0.3; done
    echo "sensor noise, one sigma of a normalized coordinate"
    for sigma in 0.002 0.005 0.008 0.012; do row "  noise $sigma" --noise "$sigma"; done
    echo "backlash, steps (the overshoot is 120)"
    for steps in 20 60 100 130 200; do row "  backlash $steps" --backlash "$steps"; done
    echo "chance that a step is skipped"
    for chance in 0.01 0.05 0.2; do row "  skip $chance" --skip "$chance"; done

# Release build of the simulator binary
[private]
_sim:
    @cargo build -q --release -p align-sim

# --- ESP32-C6 ---------------------------------------------------------------

# Build the no_std library crates for the ESP32-C6: proves they need neither std nor alloc
[group('firmware')]
check-no-std:
    cargo build {{ no_std_crates }} --target {{ target }}

# Build both firmware images in release mode (build only, never flashes)
[group('firmware')]
firmware:
    cargo build {{ firmware_crates }} --target {{ target }} --release

# Clippy on the firmware crates, warnings denied
[group('firmware')]
firmware-clippy:
    cargo clippy {{ firmware_crates }} --target {{ target }} --release -- -D warnings

# Write the flashable image of `node` or `coordinator` to target/images/ and print its size (touches no board)
[group('firmware')]
image crate: (_firmware-crate crate)
    cargo build -p align-{{ crate }} --target {{ target }} --release
    mkdir -p target/images
    espflash save-image --chip {{ chip }} {{ release_dir }}/align-{{ crate }} target/images/align-{{ crate }}.bin
    @echo "target/images/align-{{ crate }}.bin: $(wc -c < target/images/align-{{ crate }}.bin | tr -d ' ') bytes"

# --- board (each of these talks to hardware) --------------------------------
#
# With no port named, espflash chooses: ESPFLASH_PORT, then the USB devices saved in its espflash_ports.toml, then a
# prompt. Other boards are often attached to this machine; espflash never guesses among several. scripts/board.sh
# holds the flash gate and the boot log check.

# Serial ports espflash recognizes
[group('board')]
ports:
    espflash list-ports

# Chip type, revision, flash size, and MAC address of the connected board
[group('board')]
board-info port="":
    espflash board-info --chip {{ chip }} {{ if port == "" { "" } else { "--port " + port } }}

# Flash `node` or `coordinator` after its notice is acknowledged; without a terminal, check the boot log after
[group('board')]
flash crate port="" secs="15":
    {{ board }} flash {{ crate }} "{{ port }}" {{ secs }}

# Reset the board and check its boot log against the crate's notice: proves which image is running
[group('board')]
boot-log crate="node" port="" secs="10":
    {{ board }} boot-log {{ crate }} "{{ port }}" {{ secs }}

# Attach the serial monitor to a running board without flashing; the crate names the ELF used to decode addresses
[group('board')]
monitor crate="node" port="": (_firmware-crate crate)
    #!/usr/bin/env bash
    set -euo pipefail
    args=(--chip {{ chip }})
    if [ -n "{{ port }}" ]; then args+=(--port "{{ port }}"); fi
    if [ -f "{{ release_dir }}/align-{{ crate }}" ]; then args+=(--elf "{{ release_dir }}/align-{{ crate }}"); fi
    espflash monitor "${args[@]}"

# Reset the connected board over its serial line. The image restarts from boot: read its flash notice first
[group('board')]
reset port="":
    espflash reset --chip {{ chip }} {{ if port == "" { "" } else { "--port " + port } }}

# Fail with a clear message unless the argument is a firmware crate
[private]
_firmware-crate crate:
    #!/usr/bin/env bash
    case "{{ crate }}" in
      node | coordinator) ;;
      *)
        echo "the firmware crate must be 'node' or 'coordinator' (got '{{ crate }}')" >&2
        exit 1
        ;;
    esac

# --- dependencies and tools -------------------------------------------------

# License, advisory, ban, and source gate over the host and the ESP32-C6 dependency trees (deny.toml)
[group('dependencies')]
deny:
    cargo deny check

# Pinned crates against the newest release on crates.io; read docs/VERSIONS.md before moving any of them
[group('dependencies')]
outdated:
    #!/usr/bin/env bash
    set -euo pipefail
    printf '%-26s %-12s %s\n' "crate" "pinned" "newest"
    sed -n 's/^\([a-z0-9_-]*\) = { version = "=\([^"]*\)".*/\1 \2/p' Cargo.toml | while read -r name pinned; do
      newest="$(cargo search --limit 5 "$name" 2>/dev/null | sed -n "s/^$name = \"\([^\"]*\)\".*/\1/p" | head -1)"
      mark=""
      if [ -n "$newest" ] && [ "$newest" != "$pinned" ]; then mark="  <- newer"; fi
      printf '%-26s %-12s %s%s\n' "$name" "$pinned" "${newest:-unknown}" "$mark"
    done

# The tools the recipes need, with their versions, and what is missing
[group('dependencies')]
doctor:
    #!/usr/bin/env bash
    set -uo pipefail
    missing=0
    have() {
      local what="$1" need="$2"
      shift 2
      if out="$("$@" 2>/dev/null | head -1)" && [ -n "$out" ]; then
        printf '  ok       %-16s %s\n' "$what" "$out"
      else
        printf '  MISSING  %-16s %s\n' "$what" "$need"
        missing=1
      fi
    }
    have "rustc" "rustup installs the toolchain pinned in rust-toolchain.toml" rustc --version
    have "cargo" "comes with the toolchain" cargo --version
    have "rustfmt" "rustup component add rustfmt" cargo fmt --version
    have "nightly rustfmt" "rustup toolchain install nightly --component rustfmt (fmt: trailing_comma)" cargo +nightly fmt --version
    have "clippy" "rustup component add clippy" cargo clippy --version
    if rustup target list --installed 2>/dev/null | grep -qx "{{ target }}"; then
      printf '  ok       %-16s %s\n' "target" "{{ target }}"
    else
      printf '  MISSING  %-16s %s\n' "target" "rustup target add {{ target }}"
      missing=1
    fi
    have "just" "cargo install just" just --version
    have "espflash" "cargo install espflash --locked (image, flash, monitor)" espflash --version
    have "perl" "ships with macOS and most Linux systems (the board recipes)" perl -e 'print "perl $^V\n"'
    have "shasum" "ships with macOS and perl (the flash acknowledgement)" shasum --version
    have "cargo-deny" "cargo install cargo-deny --locked (deny, ci)" cargo deny --version
    have "cargo-mutants" "cargo install cargo-mutants --locked (mutants)" cargo mutants --version
    have "esp-generate" "cargo install esp-generate --locked (to regenerate the skeleton)" esp-generate --version
    if esp-generate --version 2>/dev/null | grep -q ' 1\.'; then
      note="1.x emits the previous esp-hal generation; docs/VERSIONS.md used 2.0.0"
      printf '  note     %-16s %s\n' "esp-generate" "$note"
    fi
    exit "$missing"

# --- quality ----------------------------------------------------------------

# Mutation testing of the alignment logic, the files in .cargo/mutants.toml: `just mutants -j 4` (about 5 minutes)
[group('quality')]
mutants *flags:
    {{ mutants_env }} cargo mutants --output target {{ flags }}

# Only the mutants in what changed since `base`: `just mutants-diff` (uncommitted), `just mutants-diff main`
[group('quality')]
mutants-diff base="HEAD":
    mkdir -p target
    git diff --no-ext-diff {{ base }} > target/mutants.diff
    {{ mutants_env }} cargo mutants --output target --in-diff target/mutants.diff

# Format, clippy, host tests, and the no_std build at every commit after `base`, each by that commit's own justfile
[group('quality')]
check-commits base:
    #!/usr/bin/env bash
    set -uo pipefail
    tree="$PWD/target/check-commits"
    export CARGO_TARGET_DIR="$PWD/target/check-commits-target"
    git worktree remove --force "$tree" >/dev/null 2>&1
    git worktree prune
    git worktree add --quiet --detach "$tree" HEAD || exit 1
    trap 'git worktree remove --force "$tree" >/dev/null 2>&1' EXIT
    failed=0
    for commit in $(git rev-list --reverse "{{ base }}..HEAD"); do
      git -C "$tree" checkout --quiet "$commit"
      result="ok"
      # Each commit is checked by its own justfile: the crate lists and the formatter change between commits.
      (
        cd "$tree" || exit 1
        # Separate calls: `test` takes an optional crate name and would swallow a recipe name after it.
        just fmt-check && just clippy && just test && just check-no-std
      ) >"$CARGO_TARGET_DIR.log" 2>&1 || result="FAILED"
      printf '%-7s %s\n' "$result" "$(git log -1 --format='%h %s' "$commit")"
      if [ "$result" = "FAILED" ]; then
        failed=1
        tail -n 25 "$CARGO_TARGET_DIR.log" | sed 's/^/        /'
      fi
    done
    exit "$failed"

# Everything that must pass before a commit, cheapest first
[group('quality')]
ci: fmt-check clippy test doc check-no-std firmware-clippy firmware deny

# Remove every build product, the simulator binary and the images included
[group('quality')]
clean:
    cargo clean
