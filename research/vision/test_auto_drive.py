#!/usr/bin/env python3
"""
test_auto_drive.py -- offline checks on the steering geometry. No ROS, no sim.

Covers the two fallbacks added because the lane detector goes blind on cast
shadows and the robot's answer to that was to stop:

    lidar_corridor   steer from the barrels when there is no paint
    LaneMemory       finish the bend you were on through a short dropout

Both are pure geometry, so they can be checked against paths whose answer is
known by construction instead of by watching Gazebo and hoping.

    python3 test_auto_drive.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from auto_drive import (lidar_corridor, LaneMemory, lane_centre,   # noqa: E402
                        blind_limit, should_log_idle)

FAILS = []
RUN = 0


def check(name, got, want, tol=0.0):
    global RUN
    RUN += 1
    if isinstance(want, float):
        ok = got is not None and abs(got - want) <= tol
    else:
        ok = got == want
    print(("  ok   " if ok else "  FAIL ") + f"{name}: got {got}, want {want}"
          + (f" +-{tol}" if tol else ""))
    if not ok:
        FAILS.append(name)


def wall(ys, x_lo=1.5, x_hi=3.0, step=0.5, per_band=6):
    """Lidar returns forming vertical lines at each y, across every band."""
    pts = []
    for x in np.arange(x_lo, x_hi, step):
        for k in range(per_band):
            for y in ys:
                pts.append((x + step * (k + 0.5) / per_band, y, 0.0))
    return np.array(pts, np.float32)


# ------------------------------------------------------------------ lidar
# lidar_corridor(walls, x_lo, x_hi, lane_w, side_gate, half_w, cone_r)
LC = dict(x_lo=1.5, x_hi=3.0, lane_w=3.0, side_gate=0.35, half_w=0.33,
          cone_r=0.28)


def lc(walls, **kw):
    a = dict(LC); a.update(kw)
    return lidar_corridor(walls, a["x_lo"], a["x_hi"], a["lane_w"],
                          a["side_gate"], a["half_w"], a["cone_r"],
                          **{k: v for k, v in kw.items() if k == "max_half"})


print("\n1. barrels either side -> aim down the middle")
c = lc(wall([-1.5, 1.5]))
check("got a line", len(c), 1)
check("centre", c[0][1] if c else 9.9, 0.0, 0.01)

print("\n2. SPARSE evidence still works -- this is what broke the first version")
#   four returns in the whole window, none of them sharing a 0.5 m band
sparse = np.array([[1.6, -1.4, 0.0], [2.1, 1.6, 0.0],
                   [2.7, -1.5, 0.0], [2.9, 1.5, 0.0]], np.float32)
c = lc(sparse)
check("got a line", len(c), 1)
check("centre", c[0][1] if c else 9.9, 0.05, 0.1)

print("\n3. returns all on ONE side are treated as that side's edge")
#   Both returns are to the left of the robot, so there is no right edge to
#   pair them with. band_gap then assumes a lane width inboard of the nearest
#   one -- i.e. "barrels on my left, so drive to their right" -- rather than
#   guessing that the far return is the opposite kerb. Worth pinning down,
#   because it is the behaviour whenever the robot is off to one side.
c = lc(wall([1.0, 4.0]), max_half=5.0)
check("inboard of the nearest left return", c[0][1] if c else 9.9, -0.5, 0.01)

print("\n4. only one side seen -> assume a lane width inboard of it")
c = lc(wall([1.5]))
check("got a line", len(c), 1)
check("centre is inboard", c[0][1] < 1.5 if c else False, True)

print("\n5. a barrel dead ahead is carved out, not aimed at")
c = lc(wall([-1.5, 0.0, 1.5]))
check("got a line", len(c), 1)
check("does not aim at it", abs(c[0][1]) if c else 0.0, 0.75, 0.25)

print("\n6. a corridor fully blocked by something in it is refused")
#   Edges at +-0.5 leave 0.34 m of free interval; a return dead centre carves
#   +-(cone_r + half_w) out of that, which is everything. Note two returns
#   0.9 m apart are NOT refused -- a 0.66 m robot does fit through 0.9 m, and
#   pretending otherwise would stop it on a course it could drive.
c = lc(wall([-0.5, 0.0, 0.5]))
check("no line", len(c), 0)
check("but 0.9 m clear IS accepted", len(lc(wall([-0.45, 0.45]))), 1)

print("\n7. no returns at all -> nothing, never a confident dead-ahead")
check("no line", len(lc(np.empty((0, 3), np.float32))), 0)

print("\n8. returns only outside the lookahead window are ignored")
c = lc(wall([-1.5, 1.5], x_lo=5.0, x_hi=6.0))
check("no line", len(c), 0)

# ----------------------------------------------------------------- memory
print("\n10. remembered carrot comes back unchanged from the same pose")
m = LaneMemory(max_age=3.0, min_ahead=0.8)
m.remember(100.0, 2.0, 0.0)
r = m.recall(100.5, 0.0, 0.0, 0.0)
check("x", r[0] if r else None, 2.0, 1e-6)
check("y", r[1] if r else None, 0.0, 1e-6)

print("\n11. driving 1 m toward it leaves 1 m of it")
r = m.recall(100.5, 1.0, 0.0, 0.0)
check("x", r[0] if r else None, 1.0, 1e-6)

print("\n11b. A TRAIL, not one point: recall gives the furthest still ahead")
#   This is the whole reason the single-point version was useless -- one
#   point is passed within a couple of seconds and the memory is then empty
#   at exactly the moment it is needed.
m2 = LaneMemory(max_age=10.0, min_ahead=0.8)
for i, gx in enumerate([1.0, 2.0, 3.0, 4.0, 5.0]):
    m2.remember(100.0 + i * 0.2, gx, 0.0)
check("5 points kept", len(m2.pts), 5)
check("furthest ahead from x=0", m2.recall(101.0, 0.0, 0.0, 0.0)[0], 5.0, 1e-6)
check("after driving to x=3.5", m2.recall(101.0, 3.5, 0.0, 0.0)[0], 1.5, 1e-6)
check("past the whole trail -> nothing left", m2.recall(101.0, 4.5, 0.0, 0.0), None)

print("\n11c. points too close together are merged, not hoarded")
m3 = LaneMemory(min_sep=0.3)
for i in range(20):
    m3.remember(100.0 + i * 0.01, 2.0 + i * 0.01, 0.0)
check("kept", len(m3.pts), 1)

print("\n11d. it says WHY it declined -- a silent fallback cost an hour")
m4 = LaneMemory(max_age=3.0, min_ahead=0.8)
m4.remember(100.0, 2.0, 0.0)
m4.recall(100.1, 5.0, 0.0, 0.0)          # driven well past it
check("mentions passed", "passed" in m4.why_not, True)
m4.recall(200.0, 0.0, 0.0, 0.0)          # long expired
check("mentions expired", "expired" in m4.why_not, True)

print("\n12. it expires -- memory is a bridge, not a map")
check("after 3.5 s", m.recall(103.5, 0.0, 0.0, 0.0), None)

print("\n13. once passed, it is not chased backwards")
m.remember(100.0, 2.0, 0.0)
check("robot at x=1.5, carrot 0.5 ahead", m.recall(100.1, 1.5, 0.0, 0.0), None)

print("\n14. a carrot that ends up way off to the side is dropped")
m.remember(100.0, 2.0, 0.0)
check("robot driven 3 m sideways", m.recall(100.1, 0.0, -3.0, 0.0), None)

print("\n15. rotation is handled: a point ahead becomes a point to the right")
m = LaneMemory(max_age=3.0, min_ahead=0.8)      # fresh: a trail accumulates
m.remember(100.0, 0.0, 2.0)                  # 2 m north in the global frame
r = m.recall(100.1, 0.0, 0.0, math.pi / 2)   # robot now faces north
check("x (ahead)", r[0] if r else None, 2.0, 1e-6)
check("y (centred)", r[1] if r else None, 0.0, 1e-6)
# Facing east, that same point is squarely abeam -- 0 m ahead, 2 m to the
# left. Nothing to drive toward, so it is refused rather than turned into a
# sideways lurch.
check("abeam is refused", m.recall(100.1, 0.0, 0.0, 0.0), None)

print("\n16. memory off (max_age 0) never returns anything")
m0 = LaneMemory(max_age=0.0)
m0.remember(100.0, 2.0, 0.0)
check("recall", m0.recall(100.001, 0.0, 0.0, 0.0), None)

# NOTE: the old per-band version had a tie-break that preferred the gap
# needing least steering, so two equal gaps either side of a barrel did not
# cause an arbitrary swerve. band_gap, which now does the gap search for both
# the paint and the lidar path, takes the first widest interval instead --
# the same left bias. Left alone on purpose: changing band_gap would change
# the paint behaviour the baselines are being measured against, and a dirty
# A/B is worse than a small bias. Candidate for after the comparison.

print("\n19. the blind limits: start forgiving, then stop, and cap distance")
#   Three real failures, in the order they happened:
#     a) a 1e9 "never" sentinel -> refused to start, idled 1.5 s after launch
#     b) "never sensed" meaning "never stop" -> 214 blind carrots and 216.8 m
#        dead straight, out of a 43 m arena, reported as 0% stalled
#     c) a glimpse every few seconds resets the clock while it travels far
BT, GRACE, MAXD = 4.0, 25.0, 15.0


def lim(since, dist, ever):
    return blind_limit(since, dist, ever, BT, GRACE, MAXD)


check("(a) just started, nothing sensed yet", lim(1.5, 0.0, False), "")
check("(a) still starting at 20 s", lim(20.0, 2.0, False), "")
check("(b) 30 s and NOTHING ever sensed -> stop", lim(30.0, 10.0, False) != "", True)
check("(b) the reason names the startup case",
      "ever sensed" in lim(30.0, 10.0, False), True)
check("sensed, then quiet 3 s -> carry on", lim(3.0, 1.0, True), "")
check("sensed, then quiet 5 s -> stop", lim(5.0, 1.0, True) != "", True)
check("(c) clock fine but 20 m travelled blind -> stop",
      "travelled" in lim(2.0, 20.0, True), True)
check("(c) 10 m blind is still allowed", lim(2.0, 10.0, True), "")
check("the distance cap can be disabled",
      blind_limit(2.0, 500.0, True, BT, GRACE, 0.0), "")
check("no clock at all yet", lim(None, 0.0, False), "")

print("\n21. silence must never be ambiguous between fine and stopped")
#   The bug this encodes: the driver stopped 175 s into a run and logged
#   nothing, because that reason had already been logged once at startup and
#   the dedupe swallowed the recurrence.
check("a NEW reason always logs", should_log_idle("b", "a", 0.0), True)
check("first ever reason logs", should_log_idle("a", None, 0.0), True)
check("the same reason, straight away, stays quiet",
      should_log_idle("a", "a", 0.5), False)
check("the same reason, 10 s on, repeats",
      should_log_idle("a", "a", 10.0), True)
check("...and keeps repeating", should_log_idle("a", "a", 25.0), True)
check("a tighter interval is honoured",
      should_log_idle("a", "a", 3.0, repeat_every=2.0), True)

# ------------------------------------------------- paint still wins
print("\n22. the paint path is untouched by any of this")
pts = np.array([[x, s * 1.5, 0.0] for x in np.arange(1.5, 3.0, 0.1)
                for s in (-1, 1)], np.float32)
c = lane_centre(pts, 1.5, 3.0, 0.5, 3.0, 0.35)
check("bands", len(c), 3)
check("centre", max(abs(y) for _, y in c) if c else 9.9, 0.0, 0.01)

# An earlier edit removed a whole block of checks and the suite still said
# ALL CHECKS PASSED, because passing is the absence of failure. Count them.
EXPECTED = 48          # the count today; raise it as tests are added
print(f"\n{RUN} checks run (expected at least {EXPECTED})")
if RUN < EXPECTED:
    FAILS.append(f"only {RUN} checks ran, expected {EXPECTED}+ -- did an "
                 "edit delete some?")

print("\n" + ("ALL CHECKS PASSED" if not FAILS
              else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
