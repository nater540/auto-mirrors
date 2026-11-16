#!/usr/bin/env bash
# Board actions behind the justfile `board` recipes: the flash gate, flashing, and the boot log check.
#
# Usage (the justfile is the interface; this is its back end):
#   board.sh flash <crate> <port> <secs>     gate, build, flash, then monitor or check the boot log
#   board.sh boot-log <crate> <port> <secs>  reset the board and check its boot log against the notice
#
# An empty <port> lets espflash choose, by its own rules: ESPFLASH_PORT, then its saved configuration
# (espflash_ports.toml, whose [[usb_device]] entries it matches by USB ID), and a prompt when that leaves several.
#
# The flash gate (docs/HANDOFF.md section 7): nothing is flashed until the crate's FLASH_NOTICE.txt
# has been printed and acknowledged. At a terminal the acknowledgement is typing the crate name.
# Without a terminal it is ALIGN_FLASH_ACK=<crate>:<hash>, where <hash> is the start of the SHA-256 of the notice.
# The hash ties the acknowledgement to the notice as it was read: change what the firmware drives, change the notice,
# and every old acknowledgement stops working.
#
# Without a terminal, a flash ends by capturing the boot log and checking that every line of the notice appears in
# it and that nothing panicked. That proves the image now running is the one whose notice was acknowledged.

set -euo pipefail

readonly target="riscv32imac-unknown-none-elf"
readonly chip="esp32c6"
readonly release_dir="target/$target/release"
readonly log_dir="target/boot-logs"
# How long to keep reading after every notice line has appeared, to catch a panic right after it.
readonly grace_s="1.5"

export ESPFLASH_SKIP_UPDATE_CHECK=true

die() {
  echo "$*" >&2
  exit 1
}

check_crate() {
  case "$1" in
    node | sensor | coordinator) ;;
    *) die "the firmware crate must be 'node', 'sensor', or 'coordinator' (got '$1')" ;;
  esac
}

sleep_s() {
  perl -e "select(undef, undef, undef, $1)"
}

notice_path() {
  echo "firmware/$1/FLASH_NOTICE.txt"
}

notice_hash() {
  shasum -a 256 "$(notice_path "$1")" | cut -c 1-12
}

# The port to read a boot log from without a terminal. The monitor then needs `--non-interactive`, which turns off
# espflash's own port choice, so espflash chooses first (board-info runs the same selection and needs no terminal)
# and the port it used is passed on. When its rules leave several ports, it would prompt, and without a terminal
# that fails: a port is never guessed.
capture_port() {
  local named="$1"
  if [ -n "$named" ]; then
    echo "$named"
    return
  fi
  local out port
  if ! out="$(espflash board-info --chip "$chip" </dev/null 2>&1)"; then
    printf '%s\n' "$out" | tail -n 8 | sed 's/^/    /' >&2
    die "espflash did not choose a port by itself. Name one: \`just ports\` lists them."
  fi
  port="$(sed -n "s/.*Serial port: '\(.*\)'.*/\1/p" <<<"$out" | head -n 1)"
  [ -n "$port" ] || die "espflash reached the board but did not say which port it used:"$'\n'"$out"
  echo "$port"
}

# Print the notice and require an acknowledgement of it. Returns only when flashing may go ahead.
gate() {
  local crate="$1"
  local notice hash expected
  notice="$(notice_path "$crate")"
  [ -f "$notice" ] || die "missing $notice: no image is flashed without a statement of what it does"
  hash="$(notice_hash "$crate")"
  expected="$crate:$hash"

  echo "About to flash align-$crate. What this image does to motors and I/O:"
  echo
  sed 's/^/    /' "$notice"
  echo

  if [ -n "${ALIGN_FLASH_ACK:-}" ]; then
    if [ "$ALIGN_FLASH_ACK" = "$expected" ]; then
      echo "Acknowledged by ALIGN_FLASH_ACK=$expected."
      return
    fi
    die "Not flashed: ALIGN_FLASH_ACK is '$ALIGN_FLASH_ACK', but the notice above is $expected. Read it again."
  fi
  if [ ! -t 0 ]; then
    echo "Not flashed: there is no terminal to confirm at." >&2
    die "Having read the notice above, acknowledge it with ALIGN_FLASH_ACK=$expected"
  fi
  local answer
  read -r -p "Type '$crate' to flash it, anything else to stop: " answer
  [ "$answer" = "$crate" ] || die "Not flashed."
}

# Run a command that prints the boot log, for at most `secs` seconds, into `raw`. Stops early once every notice
# line has appeared (plus a short grace). Returns 0 when the command was stopped or ended on its own.
capture() {
  local crate="$1" secs="$2" raw="$3"
  shift 3
  "$@" </dev/null >"$raw" 2>&1 &
  local pid=$! deadline
  deadline=$(($(date +%s) + secs))
  while kill -0 "$pid" 2>/dev/null; do
    if notice_seen "$crate" "$raw"; then
      sleep_s "$grace_s"
      break
    fi
    [ "$(date +%s)" -lt "$deadline" ] || break
    sleep_s 0.25
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    return 0
  fi
  # The command ended before it was stopped: espflash failed, or the monitor lost the port.
  wait "$pid"
}

# The log with color codes and carriage returns removed.
clean_log() {
  perl -pe 's/\e\[[0-9;]*m//g; s/\r//g' "$1"
}

notice_seen() {
  local crate="$1" raw="$2" line text
  text="$(clean_log "$raw")"
  while IFS= read -r line; do
    [ -n "$line" ] || continue
    grep -qF -- "$line" <<<"$text" || return 1
  done <"$(notice_path "$crate")"
}

# Capture the boot log with `"$@"`, save it, and check it. Fails unless the whole notice appeared without a panic.
capture_and_check() {
  local crate="$1" secs="$2"
  shift 2
  mkdir -p "$log_dir"
  local raw log status=0
  raw="$(mktemp "${TMPDIR:-/tmp}/align-boot.XXXXXX")"
  log="$log_dir/align-$crate-$(date -u +%Y%m%dT%H%M%SZ).log"
  capture "$crate" "$secs" "$raw" "$@" || status=$?
  clean_log "$raw" >"$log"
  rm -f "$raw"

  if [ "$status" -ne 0 ]; then
    tail -n 20 "$log" | sed 's/^/    /' >&2
    die "espflash exited with status $status before the boot log was complete. Full log: $log"
  fi
  if grep -qE 'PANIC|panicked at|Exception occurred' "$log"; then
    awk '/PANIC|panicked at|Exception occurred/ { found = 1 } found' "$log" | head -n 40 | sed 's/^/    /' >&2
    die "the image panicked at boot. Full log: $log"
  fi
  if ! notice_seen "$crate" "$log"; then
    tail -n 20 "$log" | sed 's/^/    /' >&2
    die "the boot log does not contain the notice of align-$crate within ${secs}s. Full log: $log"
  fi
  echo "Boot log checked: align-$crate is running and logged its whole notice, with no panic. Log: $log"
}

cmd_flash() {
  local crate="$1" named_port="$2" secs="$3"
  check_crate "$crate"
  gate "$crate"

  cargo build -p "align-$crate" --target "$target" --release

  local elf="$release_dir/align-$crate"
  local args=(flash --chip "$chip" --monitor)
  if [ -t 0 ] && [ -t 1 ]; then
    if [ -n "$named_port" ]; then args+=(--port "$named_port"); fi
    espflash "${args[@]}" "$elf"
  else
    local port
    port="$(capture_port "$named_port")"
    capture_and_check "$crate" "$secs" espflash "${args[@]}" --port "$port" --non-interactive "$elf"
  fi
}

cmd_boot_log() {
  local crate="$1" named_port="$2" secs="$3"
  check_crate "$crate"
  local port elf="$release_dir/align-$1"
  port="$(capture_port "$named_port")"
  local args=(monitor --chip "$chip" --port "$port" --non-interactive)
  if [ -f "$elf" ]; then args+=(--elf "$elf"); fi
  echo "Resetting the board on $port and reading its boot log (the image restarts from boot)."
  capture_and_check "$crate" "$secs" espflash "${args[@]}"
}

main() {
  local cmd="${1:-}"
  shift || true
  case "$cmd" in
    flash) cmd_flash "$1" "${2:-}" "${3:-15}" ;;
    boot-log) cmd_boot_log "$1" "${2:-}" "${3:-10}" ;;
    *) die "usage: board.sh flash|boot-log ... (see the header of this file)" ;;
  esac
}

main "$@"
