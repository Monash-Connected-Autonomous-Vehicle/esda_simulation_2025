#!/usr/bin/env python3
"""Read the ramp probe CSVs and say what separates a crossing from a stall.

    python3 analyse_ramp.py run1.csv run2.csv run3.csv

The ramp on igvc.sdf spans y = 28.3 to 30.3. For each run: did it cross,
and while it was on the ramp, was it being told to move?

    not commanded   the navigator stopped asking -- its stuck detector, or
                    no waypoint, not the slope
    traction        commanded forward, wheels reporting nothing
    moving          climbing
"""
import csv
import math
import statistics as st
import sys

RAMP_LO, RAMP_HI = 28.3, 30.3


def verdict(cmd, act, eps_cmd=0.05, eps_act=0.03):
    if abs(cmd) < eps_cmd:
        return "not commanded"
    return "traction" if abs(act) < eps_act else "moving"


def rows_of(path):
    out = []
    for r in csv.DictReader(open(path)):
        try:
            out.append({k: float(v) for k, v in r.items()})
        except ValueError:
            pass
    return out


def med(vals):
    vals = [v for v in vals if math.isfinite(v)]
    return st.median(vals) if vals else float("nan")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    print("ramp spans y = %.1f to %.1f\n" % (RAMP_LO, RAMP_HI))
    crossed = 0
    for p in sys.argv[1:]:
        rows = rows_of(p)
        if not rows:
            print("%-14s no rows" % p.split("/")[-1]); continue
        ys = [r["y"] for r in rows]
        on = [r for r in rows if RAMP_LO <= r["y"] <= RAMP_HI]
        past = [r for r in rows if r["y"] > RAMP_HI]
        ok = len(past) > 0
        crossed += ok
        print("%-16s max y %6.2f   %s   %d rows on the ramp, %d past"
              % (p.split("/")[-1], max(ys),
                 "CROSSED" if ok else "stopped short", len(on), len(past)))
        if not on:
            print("                 never reached it\n"); continue
        tally = {}
        for r in on:
            tally[verdict(r["cmd_vx"], r["act_vx"])] = \
                tally.get(verdict(r["cmd_vx"], r["act_vx"]), 0) + 1
        total = len(on)
        print("                 while on it: " + ", ".join(
            "%s %d%%" % (k, round(100 * v / total))
            for k, v in sorted(tally.items(), key=lambda kv: -kv[1])))
        print("                 pitch EKF med %+.2f deg (raw IMU %+.2f) | "
              "cmd med %.2f m/s | actual med %.2f m/s | min scan %.2f m"
              % (med([r["pitch_ekf"] for r in on]),
                 med([r["pitch_imu"] for r in on]),
                 med([r["cmd_vx"] for r in on]),
                 med([r["act_vx"] for r in on]),
                 med([r["min_scan"] for r in on])))
        flat = [r for r in rows if r["y"] < RAMP_LO]
        if flat:
            print("                 (on the flat before it: pitch EKF %+.2f, "
                  "raw IMU %+.2f)"
                  % (med([r["pitch_ekf"] for r in flat]),
                     med([r["pitch_imu"] for r in flat])))
        print()
    print("crossed %d of %d" % (crossed, len(sys.argv) - 1))
    print()
    print("if the stalls are mostly 'not commanded', the navigator's stuck")
    print("detector is firing on the slope and the fix is in its thresholds.")
    print("if they are mostly 'traction', the robot cannot climb and the fix")
    print("is in the controller or the wheel physics.")


if __name__ == "__main__":
    main()
