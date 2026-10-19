#!/usr/bin/env python3
"""Offline checks for the lane-side hypothesis test. No ROS, no model."""
import lane_side as LS

W = 3.0          # lane width the driver assumes
FOV = 90.0       # camera field of view

FAILS = []
RUN = 0


def check(name, got, want, tol=1e-6):
    global RUN
    RUN += 1
    ok = abs(got - want) <= tol if isinstance(want, float) else got == want
    print("  %-62s got %-22s want %s   %s"
          % (name, round(got, 3) if isinstance(got, float) else got,
             round(want, 3) if isinstance(want, float) else want,
             "ok" if ok else "FAIL"))
    if not ok:
        FAILS.append(name)


def centre(y, on_left, x):
    (lo, hi), flipped = LS.one_sided_lane(y, on_left, W, LS.view_half_at(x, FOV))
    return 0.5 * (lo + hi), flipped


print("=" * 78)
print("1. THE TRAP: outside the lane, its right edge close on the left")
print("=" * 78)
# Robot has crossed the right-hand line and sits 0.8 m outside it. At x=2 m a
# 90 deg camera sees +-2.0 m, so A's implied second line at 0.8-3.0 = -2.2 m
# is OUTSIDE the window -> A is not refutable there. Push the band further out
# where it IS testable.
c, f = centre(0.8, True, 3.0)      # view_half 3.0 m at x=3
check("flips to 'this is the right edge'", f, True)
check("steers LEFT, back across the line", c > 0.8, True)
check("targets the lane centre beyond it", c, 0.8 + W / 2)

print()
print("=" * 78)
print("2. HEALTHY: inside the lane, near line lost to the camera's near limit")
print("=" * 78)
# Robot hugging the right line, so the LEFT line is far (2.7 m of a 3 m lane).
# The gate (near < w/2) does not open, A stands, and it steers left toward
# the centre -- which is correct.
c, f = centre(2.7, True, 3.0)
check("does not flip", f, False)
check("keeps the old interpretation", c, 2.7 - W / 2)
check("still steers back toward the lane", c > 0.0, True)

print()
print("=" * 78)
print("3. NORMAL: line close on the left, second line genuinely out of view")
print("=" * 78)
# Near band, x=1 m: the camera only sees +-1.0 m, so NEITHER hypothesis is
# testable and the old behaviour must stand.
c, f = centre(0.9, True, 1.0)
check("no flip when nothing is testable", f, False)
check("old behaviour preserved", c, 0.9 - W / 2)

print()
print("=" * 78)
print("4. DISABLED: view_half 0 restores the original code exactly")
print("=" * 78)
for y, on_left in ((0.8, True), (2.7, True), (-0.8, False), (-2.7, False)):
    (lo, hi), f = LS.one_sided_lane(y, on_left, W, 0.0)
    want = (y - W, y) if on_left else (y, y + W)
    check("y=%+.1f %-5s interval unchanged" % (y, "left" if on_left else "right"),
          (lo, hi), want)
    check("y=%+.1f %-5s never flips" % (y, "left" if on_left else "right"), f, False)

print()
print("=" * 78)
print("5. MIRRORED: the same trap with the line on the RIGHT")
print("=" * 78)
c, f = centre(-0.8, False, 3.0)
check("flips", f, True)
check("steers RIGHT, back across the line", c < -0.8, True)
check("symmetric with the left-hand case", c, -(0.8 + W / 2))

print()
print("=" * 78)
print("6. view_half_at is the camera's actual geometry")
print("=" * 78)
check("90 deg at 2 m sees +-2.00 m", LS.view_half_at(2.0, 90.0), 2.0, 1e-9)
check("90 deg at 4 m sees +-4.00 m", LS.view_half_at(4.0, 90.0), 4.0, 1e-9)
check("62.4 deg at 2 m sees only +-1.21 m", LS.view_half_at(2.0, 62.4), 1.2106, 1e-3)
check("fov 0 disables", LS.view_half_at(2.0, 0.0), 0.0, 1e-9)

EXPECTED = 20
print("\n%d checks run (expected at least %d)" % (RUN, EXPECTED))
if RUN < EXPECTED:
    FAILS.append("only %d checks ran, expected %d+ -- did an edit delete some?" % (RUN, EXPECTED))
print()
if FAILS:
    print("FAILURES:")
    for f in FAILS:
        print("  -", f)
    raise SystemExit(1)
print("ALL CHECKS PASSED")
