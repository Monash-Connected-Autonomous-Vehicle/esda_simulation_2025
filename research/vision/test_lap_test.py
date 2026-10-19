#!/usr/bin/env python3
"""
test_lap_test.py -- offline checks on the lap scorer, no ROS and no simulator.

A benchmark that is wrong is worse than no benchmark, because it is believed.
These feed LapTracker synthetic paths whose answer is known by construction --
a loop, an out-and-back, a tight circle near the start, a figure of eight, a
run with a long stall -- and assert what it should and should not call a lap.

    python3 test_lap_test.py
"""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lap_test import LapTracker                                  # noqa: E402

HZ = 20.0
FAILS = []


def check(name, got, want, tol=0.0):
    ok = (abs(got - want) <= tol) if isinstance(want, float) else (got == want)
    print(("  ok   " if ok else "  FAIL ") + f"{name}: got {got}, want {want}"
          + (f" +-{tol}" if tol else ""))
    if not ok:
        FAILS.append(name)


# ------------------------------------------------------------------ paths
def feed(tr, pts, v=None, t0=0.0):
    """pts is [(x, y, yaw)] at HZ; returns the tracker."""
    for i, (x, y, yaw) in enumerate(pts):
        tr.update(t0 + i / HZ, x, y, yaw, v)
    tr.finish()
    return tr


def circle(r, loops, speed=1.0, ccw=True):
    """Start at the origin, on the circle, centre at (0, r). Yaw is tangent."""
    n = int(2 * math.pi * r * loops / speed * HZ)
    out = []
    for i in range(n + 1):
        th = 2 * math.pi * loops * i / n
        s = 1.0 if ccw else -1.0
        out.append((r * math.sin(th) * s, r * (1 - math.cos(th)), th * s))
    return out


def straight(x0, y0, x1, y1, speed=1.0):
    d = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(d / speed * HZ))
    yaw = math.atan2(y1 - y0, x1 - x0)
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw)
            for i in range(n + 1)]


def uturn(cx, cy, r, yaw0, speed=1.0):
    """Half circle, turning left, ending reversed."""
    n = max(1, int(math.pi * r / speed * HZ))
    out = []
    for i in range(n + 1):
        a = math.pi * i / n
        out.append((cx + r * math.sin(a + yaw0 - math.pi / 2) * 0 + r * math.cos(yaw0 - math.pi / 2 + a),
                    cy + r * math.sin(yaw0 - math.pi / 2 + a),
                    yaw0 + a))
    return out


# ------------------------------------------------------------------ tests
print("\n1. one clean loop of a 10 m circle")
tr = feed(LapTracker(), circle(10, 1))
check("laps", len(tr.laps), 1)
check("path length", tr.path_len, 2 * math.pi * 10, 0.5)
check("net turn", tr.turn_net, 2 * math.pi, 0.1)
check("max dist from start", tr.max_dist, 20.0, 0.2)

print("\n2. two loops -> two laps; the first reads short, as it should")
# The lap closes the moment the robot enters the --close circle, which on a
# 10 m circle happens about 4 m of arc before the exact start -- a finish gate
# has width. The second lap is measured from there, so it comes out full.
tr = feed(LapTracker(), circle(10, 2))
check("laps", len(tr.laps), 2)
if len(tr.laps) == 2:
    check("lap 1 length (short by the gate)", 55.0 < tr.laps[0][2] < 63.0, True)
    check("lap 2 length", tr.laps[1][2], 62.8, 1.5)

print("\n3. three quarters of a loop is NOT a lap")
tr = feed(LapTracker(), circle(10, 0.75))
check("laps", len(tr.laps), 0)
check("still records progress", round(tr.path_len) > 40, True)

print("\n4. out 20 m, U-turn, back again -- 40 m and returns, but NOT a lap")
path = (straight(0, 0, 20, 0)
        + uturn(20, 1, 1, 0.0)
        + straight(20, 2, 0, 2))
tr = feed(LapTracker(), path)
check("departed", tr.max_dist > 6, True)
check("path length", round(tr.path_len) >= 30, True)
check("came back", tr.dist_from_start < 4, True)
check("net turn is about pi", abs(tr.turn_net), math.pi, 0.2)
check("laps", len(tr.laps), 0)

print("\n5. ...and the same path IS scored a lap if the turn term is off")
tr = feed(LapTracker(min_turn=0.0), path)
check("laps with --min-turn 0", len(tr.laps), 1)

print("\n6. five tight loops near the start never leave the depart circle")
tr = feed(LapTracker(), circle(2.5, 5))
check("path length", tr.path_len, 2 * math.pi * 2.5 * 5, 1.0)
check("net turn", tr.turn_net, 10 * math.pi, 0.2)
check("max dist", tr.max_dist, 5.0, 0.2)
check("laps", len(tr.laps), 0)

print("\n7. figure of eight, started at the crossing: each lobe is a real loop")
def fig8_from_crossing(r=8.0, speed=1.0):
    n = int(2 * math.pi * r / speed * HZ)
    a = [(r * math.sin(2 * math.pi * i / n), r * (1 - math.cos(2 * math.pi * i / n)),
          2 * math.pi * i / n) for i in range(n + 1)]
    b = [(r * math.sin(2 * math.pi * i / n), -r * (1 - math.cos(2 * math.pi * i / n)),
          -2 * math.pi * i / n) for i in range(n + 1)]
    return a, b

a, b = fig8_from_crossing()
tr = feed(LapTracker(), a + b)
check("net turn over the whole eight is ~0", abs(tr.turn_net) < 0.5, True)
check("total turning about 4 pi", tr.turn_abs, 4 * math.pi, 0.3)
check("laps (default): one per lobe", len(tr.laps), 2)

print("\n8. ...but started mid-lobe, one circuit nets zero and is rejected")
# Begin a quarter of the way round the upper lobe, so the start is NOT the
# crossing. Finish the upper lobe, do the lower one, then close the upper.
q = len(a) // 4
mid = a[q:] + b + a[:q + 1]
tr = feed(LapTracker(), mid)
check("came back to the start", tr.dist_from_start < 1.0, True)
check("net turn ~0", abs(tr.turn_net) < 0.5, True)
check("laps (default)", len(tr.laps), 0)
tr = feed(LapTracker(min_turn=0.0), mid)
check("laps (--min-turn 0)", len(tr.laps), 1)

print("\n9. a 60 s stall in the middle is measured")
pts = straight(0, 0, 10, 0) + [(10.0, 0.0, 0.0)] * int(60 * HZ) \
    + straight(10, 0, 20, 0)
tr = LapTracker()
for i, (x, y, yaw) in enumerate(pts):
    moving = not (len(straight(0, 0, 10, 0)) <= i
                  < len(straight(0, 0, 10, 0)) + int(60 * HZ))
    tr.update(i / HZ, x, y, yaw, 1.0 if moving else 0.0)
tr.finish()
check("stall count", len(tr.stalls), 1)
if tr.stalls:
    check("stall duration", tr.stalls[0][1], 60.0, 0.2)
check("stalled seconds", tr.stalled_s, 60.0, 0.2)

print("\n10. a half-second pause is below --stall-min and ignored")
tr = LapTracker()
for i in range(int(20 * HZ)):
    v = 0.0 if 100 <= i < 110 else 1.0
    tr.update(i / HZ, i / HZ, 0.0, 0.0, v)
tr.finish()
check("stall count", len(tr.stalls), 0)

print("\n11. a stall still running at the end is not dropped")
tr = LapTracker()
for i in range(int(30 * HZ)):
    tr.update(i / HZ, min(i, 100) / HZ, 0.0, 0.0, 1.0 if i < 100 else 0.0)
check("before finish()", len(tr.stalls), 0)
tr.finish()
check("after finish()", len(tr.stalls), 1)

print("\n12. close >= depart is refused rather than silently scoring nonsense")
try:
    LapTracker(depart=4.0, close=6.0)
    check("raised", False, True)
except ValueError:
    check("raised ValueError", True, True)

print("\n13. a sample with a clock that went backwards is ignored")
tr = LapTracker()
tr.update(0.0, 0.0, 0.0, 0.0, 1.0)
tr.update(1.0, 1.0, 0.0, 0.0, 1.0)
before = tr.path_len
tr.update(0.5, 50.0, 0.0, 0.0, 1.0)
check("path unchanged", tr.path_len, before, 1e-9)

print("\n14. a lap just under the distance floor does not count")
tr = feed(LapTracker(min_lap=70.0), circle(10, 1))     # 62.8 m < 70
check("laps", len(tr.laps), 0)

print("\n15. yaw can be omitted and taken from the track")
pts = [(x, y, None) for x, y, _ in circle(10, 1)]
tr = LapTracker()
for i, (x, y, _) in enumerate(pts):
    tr.update(i / HZ, x, y, None, 1.0)
tr.finish()
check("laps", len(tr.laps), 1)
check("net turn", tr.turn_net, 2 * math.pi, 0.2)

print("\n" + ("ALL CHECKS PASSED" if not FAILS
              else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
