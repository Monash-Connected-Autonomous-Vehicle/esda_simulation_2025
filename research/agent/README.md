# agent/ — the job queue

`bash agent.sh` in WSL watches `agent/queue/` and runs any `.job` file that
appears there, oldest name first.

Why: every lap run needs a keystroke on the machine the simulator lives on,
and a tuning loop is dozens of runs. Starting this once means the runs can be
queued by anything that can write into `C:\work\felix\mcav\for_server` — which
includes Claude, who can then read the reports back out of `logs/` and queue
the next experiment without you doing anything.

| path | what it is |
|---|---|
| `queue/*.job` | waiting to run. A plain bash script. Delete one to cancel it. |
| `running/` | the job executing right now |
| `done/*.job` | exactly what ran, kept for the record |
| `done/*.status` | `exit=<code> after <n>s` |
| `logs/*.log` | the job script, then all its output |
| `status.txt` | one file to read: state, current job, queue, recent results |
| `STOP` | `touch agent/STOP` to stop after tidying up |

Nothing is hidden: each job's full script is copied into the top of its own
log before it runs, and kept in `done/` afterwards.

A job gets 90 minutes (`JOB_TIMEOUT`), then it and every process it started are
killed and the stack is torn down, so one wedged run cannot block the queue or
poison the next measurement.

## Lap benchmark reports

Jobs here mostly call `./demo.sh lap <world>`, which writes
`logs/run_<world>_<stamp>/` containing:

- `lap_<world>_<stamp>.txt` — the human summary (PASS/FAIL, distance, stalls)
- `lap_<world>_<stamp>.json` — the same, machine-readable, plus the full track
- `sim.log`, `nav.log`, `vision.log`, `drive.log` — every stage's output

`logs/lap_latest.txt` is always the most recent summary.
