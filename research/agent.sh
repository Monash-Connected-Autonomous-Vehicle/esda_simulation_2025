#!/usr/bin/env bash
# =====================================================================
#  agent.sh — a job queue for the ESDA stack.
#
#      bash agent.sh            run until stopped (Ctrl-C, or agent/STOP)
#      bash agent.sh --once     run whatever is queued, then exit
#      bash agent.sh --status   print the queue and exit
#
#  WHY THIS EXISTS
#
#  Tuning a navigation stack is a loop: run the course, read what went
#  wrong, change one number, run it again. Each run is twenty minutes of
#  waiting and one keystroke of work, and the keystroke has to happen on
#  this machine, because the simulator lives in WSL here.
#
#  So: you start this once. It watches agent/queue/ and runs any .job file
#  that appears there, in name order, logging everything. Runs can then be
#  queued by dropping a file into that folder from anywhere that can write
#  to C:\work\felix\mcav\for_server — including Claude, which can read the
#  reports back out of logs/ afterwards and queue the next experiment.
#
#  WHAT RUNS
#
#  A .job file is a bash script. Nothing is hidden: every job is printed
#  into its own log before it executes and is kept in agent/done/ after,
#  so you can read exactly what ran, in order, at any point. Delete a file
#  from agent/queue/ to cancel it before it starts.
#
#  STOPPING
#
#  Ctrl-C, or `touch agent/STOP` from anywhere. A job already running is
#  given 10 s to tidy up, then killed, and the stack is torn down.
#
#  To abandon just ONE job and keep going: `touch agent/SKIP`. Worth having --
#  a wedged run should not cost the whole queue, and the first real run hung
#  for eighteen minutes inside a ros2 call that could not time out.
# =====================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
A="$HERE/agent"
Q="$A/queue"; R="$A/running"; D="$A/done"; L="$A/logs"
mkdir -p "$Q" "$R" "$D" "$L"
export ESDA_HOME="$HERE"                    # jobs cd here
JOB_TIMEOUT="${JOB_TIMEOUT:-5400}"          # 90 min, so one bad job cannot wedge the queue
POLL="${POLL:-5}"

say()  { echo -e "\033[1;36m==> $*\033[0m"; }
ok()   { echo -e "    \033[1;32mOK\033[0m  $*"; }
warn() { echo -e "    \033[1;33m!!\033[0m  $*"; }

CHILD=""
cleanup() {
  echo
  warn "stopping"
  if [ -n "$CHILD" ] && kill -0 "$CHILD" 2>/dev/null; then
    kill -TERM "-$CHILD" 2>/dev/null || kill -TERM "$CHILD" 2>/dev/null
    sleep 10
    kill -KILL "-$CHILD" 2>/dev/null || kill -KILL "$CHILD" 2>/dev/null
  fi
  bash "$HERE/demo.sh" down >/dev/null 2>&1
  status_file "stopped" "-"
  exit 0
}
trap cleanup INT TERM

status_file() {
  # One file to read to know what is going on, for a human or for Claude.
  {
    echo "state:    $1"
    echo "job:      $2"
    echo "updated:  $(date '+%Y-%m-%d %H:%M:%S')"
    echo "pid:      $$"
    echo "queued:   $(ls -1 "$Q"/*.job 2>/dev/null | wc -l)"
    echo "done:     $(ls -1 "$D"/*.job 2>/dev/null | wc -l)"
    echo "--- queue ---"
    ls -1 "$Q" 2>/dev/null
    echo "--- last 10 finished ---"
    ls -1t "$D"/*.status 2>/dev/null | head -10 | while read -r f; do
      echo "  $(basename "${f%.status}")  ->  $(cat "$f")"
    done
  } > "$A/status.txt" 2>/dev/null
}

print_status() { status_file "query" "-"; cat "$A/status.txt"; }

case "${1:-}" in
  --status) print_status; exit 0 ;;
esac
ONCE=0; [ "${1:-}" = "--once" ] && ONCE=1

rm -f "$A/STOP" "$A/SKIP"

# A job left in running/ means the last runner died or was interrupted part
# way through it. Put it back at the front of the queue rather than silently
# dropping it -- otherwise an interrupted run quietly vanishes from the plan
# and the gap is only noticed when the reports are compared at the end.
for orphan in "$R"/*.job; do
  [ -e "$orphan" ] || continue
  warn "requeueing $(basename "$orphan" .job), left over from an interrupted run"
  mv "$orphan" "$Q/" 2>/dev/null
done

say "ESDA job runner"
echo "    queue:   $Q"
echo "    logs:    $L"
echo "    stop:    Ctrl-C here, or  touch $A/STOP"
echo "    timeout: ${JOB_TIMEOUT}s per job"
[ "$ONCE" = "1" ] && echo "    (--once: will exit when the queue is empty)"
echo

while true; do
  [ -f "$A/STOP" ] && { rm -f "$A/STOP"; cleanup; }

  job="$(ls -1 "$Q"/*.job 2>/dev/null | head -1)"
  if [ -z "$job" ]; then
    status_file "idle" "-"
    [ "$ONCE" = "1" ] && { ok "queue empty, exiting"; break; }
    sleep "$POLL"
    continue
  fi

  name="$(basename "$job" .job)"
  mv "$job" "$R/$name.job" 2>/dev/null || continue
  log="$L/$name.log"
  status_file "running" "$name"

  say "JOB $name   ($(date '+%H:%M:%S'))"
  {
    echo "=============================================================="
    echo " job:     $name"
    echo " started: $(date '+%Y-%m-%d %H:%M:%S')"
    echo " host:    $(hostname)"
    echo "=============================================================="
    echo "--- script ---------------------------------------------------"
    cat "$R/$name.job"
    echo "--- output ---------------------------------------------------"
  } > "$log"

  setsid bash "$R/$name.job" >> "$log" 2>&1 &
  CHILD=$!
  elapsed=0
  while kill -0 "$CHILD" 2>/dev/null; do
    sleep 2; elapsed=$((elapsed + 2))
    if [ -f "$A/STOP" ]; then rm -f "$A/STOP"; cleanup; fi
    if [ -f "$A/SKIP" ]; then
      # Abandon THIS job and carry on with the queue. STOP ends everything,
      # which costs a keystroke to restart; a single wedged run should not.
      rm -f "$A/SKIP"
      warn "SKIP requested -- killing $name and moving on"
      kill -TERM "-$CHILD" 2>/dev/null || kill -TERM "$CHILD" 2>/dev/null
      sleep 5
      kill -KILL "-$CHILD" 2>/dev/null || kill -KILL "$CHILD" 2>/dev/null
      echo "[agent] SKIPPED on request after ${elapsed}s" >> "$log"
      break
    fi
    if [ "$elapsed" -ge "$JOB_TIMEOUT" ]; then
      warn "job $name hit the ${JOB_TIMEOUT}s timeout -- killing it"
      kill -TERM "-$CHILD" 2>/dev/null; sleep 10
      kill -KILL "-$CHILD" 2>/dev/null
      echo "[agent] KILLED after ${JOB_TIMEOUT}s" >> "$log"
      break
    fi
  done
  wait "$CHILD" 2>/dev/null; rc=$?
  CHILD=""

  echo "exit=$rc after ${elapsed}s" > "$D/$name.status"
  mv "$R/$name.job" "$D/$name.job" 2>/dev/null
  echo "[agent] finished rc=$rc after ${elapsed}s" >> "$log"
  if [ "$rc" = "0" ]; then ok "$name finished (${elapsed}s)"
  else warn "$name exited $rc (${elapsed}s) -- see $log"; fi

  # Never leave a half-dead stack behind for the next job to trip over.
  bash "$HERE/demo.sh" down >/dev/null 2>&1
  status_file "idle" "-"
  sleep 2
done
status_file "stopped" "-"
