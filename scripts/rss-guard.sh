#!/usr/bin/env bash
# Runs a command and kills any process under it whose resident memory passes a limit.
#
# Usage: rss-guard.sh <limit-mb> <log-file> <command> [args...]
#
# Why: a mutant can turn a bounded loop into an endless one that appends to a buffer, and a test timeout of minutes
# is time enough to fill all of memory. On 2026-11-18 two such mutants in bench-rig reached 12 and 5 GB under
# `cargo mutants -j 2` and took the machine's login session down. macOS does not enforce `ulimit -v` or `ulimit -d`
# (setting them fails), so the limit is watched from outside.
#
# A killed test reads to cargo-mutants as a failed test, so its mutant counts as caught. Every kill is printed to
# standard error and appended to <log-file>, and listed again when the command ends: read them, since a mutant that
# ran away is caught by the guard, not by a test.

set -uo pipefail

if [[ $# -lt 3 ]]; then
  echo "usage: rss-guard.sh <limit-mb> <log-file> <command> [args...]" >&2
  exit 2
fi
readonly limit_mb="$1"
readonly log_file="$2"
shift 2
readonly limit_kb=$((limit_mb * 1024))
readonly poll_s="0.5"

mkdir -p "$(dirname "$log_file")"
: > "$log_file"

"$@" &
readonly root=$!
trap 'kill -TERM "$root" 2>/dev/null' INT TERM

# Prints "pid rss_kb command" for each descendant of $root over the limit.
over_limit() {
  ps -A -o pid=,ppid=,rss=,command= | awk -v root="$root" -v limit="$limit_kb" '
    {
      pid = $1; parent[pid] = $2; rss[pid] = $3
      $1 = ""; $2 = ""; $3 = ""; command[pid] = substr($0, 4)
    }
    END {
      for (pid in parent) {
        if (rss[pid] <= limit) continue
        for (up = parent[pid]; up != "" && up > 1; up = parent[up]) {
          if (up == root) { print pid, rss[pid], substr(command[pid], 1, 200); break }
        }
      }
    }'
}

while kill -0 "$root" 2>/dev/null; do
  while read -r pid rss_kb command; do
    [[ -z "$pid" ]] && continue
    kill -KILL "$pid" 2>/dev/null || continue
    line="rss-guard: killed pid $pid at $((rss_kb / 1024)) MB (limit $limit_mb MB): $command"
    echo "$line" >&2
    echo "$line" >> "$log_file"
  done < <(over_limit)
  sleep "$poll_s"
done

wait "$root"
status=$?
if [[ -s "$log_file" ]]; then
  echo "rss-guard: $(wc -l < "$log_file" | tr -d ' ') process(es) killed for memory; their mutants count as caught:" >&2
  cat "$log_file" >&2
fi
exit "$status"
