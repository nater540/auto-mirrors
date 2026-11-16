#!/usr/bin/env bash
# Board actions behind the justfile `board` recipes: finding the port, the flash gate, flashing, and the boot log.
#
# Usage (the justfile is the interface; this is its back end):
#   board.sh port <port-or-empty>                     print the port to use, or fail
#   board.sh ports                                    list serial ports, Espressif ones first
#   board.sh flash <crate> <port> <secs>              gate, build, flash, then monitor or check the boot log
#   board.sh boot-log <crate> <port> <secs>           reset the board and check its boot log against the notice
#
# The flash gate (docs/HANDOFF.md section 7): nothing is flashed until the crate's FLASH_NOTICE.txt has been
# printed and acknowledged. At a terminal the acknowledgement is typing the crate name. Without a terminal it is
# ALIGN_FLASH_ACK=<crate>:<hash>, where <hash> is the start of the SHA-256 of the notice. The hash ties the
# acknowledgement to the notice as it was read: change what the firmware drives, change the notice, and every old
# acknowledgement stops working.
#
# Without a terminal, a flash ends by capturing the boot log and checking that every line of the notice appears in
# it and that nothing panicked. That proves the image now running is the one whose notice was acknowledged.

set -euo pipefail

readonly target="riscv32imac-unknown-none-elf"
readonly chip="esp32c6"
readonly release_dir="target/$target/release"
readonly log_dir="target/boot-logs"
# The USB vendor ID of Espressif. The built-in USB-Serial-JTAG of the ESP32-C6 enumerates with it.
readonly espressif_vid="303A"
# How long to keep reading after every notice line has appeared, to catch a panic right after it.
readonly grace_s="1.5"

export ESPFLASH_SKIP_UPDATE_CHECK=true

die() {
  echo "$*" >&2
  exit 1
}

check_crate() {
  case "$1" in
    node | coordinator) ;;
    *) die "the firmware crate must be 'node' or 'coordinator' (got '$1')" ;;
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

# The Espressif USB serial ports, one per line. On macOS each device appears twice, as /dev/cu.* and /dev/tty.*;
# only the cu device is kept, because opening the tty device waits for a carrier that never comes.
espressif_ports() {
  espflash list-ports 2>/dev/null \
    | awk -v vid="$espressif_vid" 'toupper($2) ~ (":" vid "$") && $1 !~ /^\/dev\/tty\./ { print $1 }'
}

# The port to use: the one named, or else the only Espressif port attached. Other boards are often attached to
# this machine, so a port is never guessed among several, and a non-Espressif port is never picked automatically.
resolve_port() {
  local named="$1"
  if [ -n "$named" ]; then
    [ -e "$named" ] || die "no such port: $named"
    echo "$named"
    return
  fi
  local ports count
  ports="$(espressif_ports)"
  count="$(printf '%s' "$ports" | grep -c . || true)"
  case "$count" in
    1) echo "$ports" ;;
    0) die "no Espressif USB serial port is attached. \`just ports\` lists what is." ;;
    *)
      echo "several Espressif ports are attached; name one (\`just ports\` lists them):" >&2
      printf '  %s\n' $ports >&2
      exit 1
      ;;
  esac
}

cmd_ports() {
  local espressif
  espressif="$(espressif_ports)"
  echo "Espressif USB serial ports (picked automatically when there is exactly one):"
  if [ -n "$espressif" ]; then printf '  %s\n' $espressif; else echo "  none"; fi
  echo
  echo "Everything espflash sees, other boards included (never picked automatically):"
  espflash list-ports --list-all-ports 2>/dev/null | sed 's/^/  /' || true
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

  local port
  port="$(resolve_port "$named_port")"
  cargo build -p "align-$crate" --target "$target" --release

  local elf="$release_dir/align-$crate"
  local args=(flash --chip "$chip" --port "$port" --monitor)

  echo "Port: $port"
  if [ -t 0 ] && [ -t 1 ]; then
    espflash "${args[@]}" "$elf"
  else
    capture_and_check "$crate" "$secs" espflash "${args[@]}" --non-interactive "$elf"
  fi
}

cmd_boot_log() {
  local crate="$1" named_port="$2" secs="$3"
  check_crate "$crate"
  local port elf="$release_dir/align-$1"
  port="$(resolve_port "$named_port")"
  local args=(monitor --chip "$chip" --port "$port" --non-interactive)
  if [ -f "$elf" ]; then args+=(--elf "$elf"); fi
  echo "Resetting the board on $port and reading its boot log (the image restarts from boot)."
  capture_and_check "$crate" "$secs" espflash "${args[@]}"
}

main() {
  local cmd="${1:-}"
  shift || true
  case "$cmd" in
    port) resolve_port "${1:-}" ;;
    ports) cmd_ports ;;
    flash) cmd_flash "$1" "${2:-}" "${3:-15}" ;;
    boot-log) cmd_boot_log "$1" "${2:-}" "${3:-10}" ;;
    *) die "usage: board.sh port|ports|flash|boot-log ... (see the header of this file)" ;;
  esac
}

main "$@"
