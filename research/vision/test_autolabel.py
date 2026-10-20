#!/usr/bin/env python3
"""
test_autolabel.py -- is the computed label actually where the thing is?

An auto-labelled dataset is worth less than no dataset if the geometry is
subtly wrong: the model trains happily on confident nonsense and nobody finds
out until the robot drives into a barrel. The whole chain is

    texture pixel <-> world metres <-> base_link metres <-> image pixel

and a sign error anywhere in it still produces plausible-looking masks. So
these tests work backwards from ground truth: take a point we KNOW is paint
(or a barrel, from the SDF), put the robot at a known pose relative to it,
and assert the label lands on the pixel the projection says it should.

Needs the generated worlds and their textures; run from the repo.

    python3 test_autolabel.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autolabel as al                                           # noqa: E402
import synth_gen as sg                                           # noqa: E402
import cv2                                                       # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
WORLD = "igvc_oval_test"
TEX = os.path.join(REPO, "esda_simulation_2025", "models", "orange_igvc",
                   "materials", "textures", f"ground_{WORLD[5:]}.png")
SDF = os.path.join(REPO, "esda_simulation_2025", "worlds", f"{WORLD}.sdf")
FAILS = []


def check(name, got, want, tol=0.0):
    ok = (abs(got - want) <= tol) if isinstance(want, float) else (got == want)
    print(("  ok   " if ok else "  FAIL ") + f"{name}: got {got}, want {want}"
          + (f" +-{tol}" if tol else ""))
    if not ok:
        FAILS.append(name)


print(f"loading {os.path.basename(TEX)}")
paint = al.paint_mask(cv2.imread(TEX, cv2.IMREAD_COLOR))
barrels = al.barrels_from_sdf(SDF)
pix = sg.pixel_to_ground(0.0)
print(f"  paint covers {paint.mean()*100:.1f}% of the texture, "
      f"{len(barrels)} barrels")

print("\n1. w2p round-trips through the texture")
for (X, Y) in [(0.0, 0.0), (10.0, -5.0), (-15.0, 12.0)]:
    u, v = al.w2p(X, Y)
    bx = al.X_MIN + u / al.TEX_W * (al.X_MAX - al.X_MIN)
    by = al.Y_MAX - v / al.TEX_H * (al.Y_MAX - al.Y_MIN)
    check(f"({X},{Y}) -> px -> world x", bx, X, 1e-6)
    check(f"({X},{Y}) -> px -> world y", by, Y, 1e-6)

print("\n2. the v flip is the right way up")
#   Top row of the texture must be the LARGEST world y, not the smallest.
_, v_top = al.w2p(0.0, al.Y_MAX)
_, v_bot = al.w2p(0.0, al.Y_MIN)
check("y = +18.5 is row 0", float(v_top), 0.0, 1e-6)
check("y = -18.5 is the last row", float(v_bot), float(al.TEX_H), 1e-6)

print("\n3. A PAINTED POINT 3 m AHEAD LANDS ON THE LANE MASK")
#   The test that catches a sign error anywhere in the chain.
ys, xs = np.nonzero(paint)
rng = np.random.default_rng(7)
hits = tries = 0
for k in rng.choice(len(xs), 40, replace=False):
    u_t, v_t = float(xs[k]), float(ys[k])
    wx = al.X_MIN + u_t / al.TEX_W * (al.X_MAX - al.X_MIN)
    wy = al.Y_MAX - v_t / al.TEX_H * (al.Y_MAX - al.Y_MIN)
    if not (al.X_MIN + 5 < wx < al.X_MAX - 5):
        continue
    pose = (wx - 3.0, wy, 0.0)            # 3 m behind it, facing +x
    m = al.label_frame(paint, barrels, pose, 0.0, pix)
    u_i, v_i = sg.ground_to_pixel(3.0, 0.0, 0.0)
    ui, vi = int(round(float(u_i))), int(round(float(v_i)))
    if not (0 <= ui < sg.W and 0 <= vi < sg.H_IMG):
        continue
    tries += 1
    # 2 px of slack: the texture is ~3 cm per pixel and we rounded twice.
    # A barrel standing on the line is a legitimate miss -- paint behind a
    # barrel is not visible paint, and labelling it would teach the model to
    # hallucinate a lane through an obstacle. Both of the misses this test
    # found on first run were exactly that, one with a barrel 0.34 m from
    # the point, so occlusion counts as correct rather than as a failure.
    win = (slice(max(0, vi-2), vi+3), slice(max(0, ui-2), ui+3))
    hits += bool(m["lane"][win].any() or m["cone"][win].any())
check("painted points labelled lane, or correctly occluded", hits, tries)

print("\n4. ...and it still works with the robot rotated")
for yaw in (math.pi / 2, math.pi, -math.pi / 3):
    k = rng.choice(len(xs))
    wx = al.X_MIN + float(xs[k]) / al.TEX_W * (al.X_MAX - al.X_MIN)
    wy = al.Y_MAX - float(ys[k]) / al.TEX_H * (al.Y_MAX - al.Y_MIN)
    pose = (wx - 3.0 * math.cos(yaw), wy - 3.0 * math.sin(yaw), yaw)
    m = al.label_frame(paint, barrels, pose, 0.0, pix)
    u_i, v_i = sg.ground_to_pixel(3.0, 0.0, 0.0)
    ui, vi = int(round(float(u_i))), int(round(float(v_i)))
    check(f"yaw={yaw:+.2f}", bool(m["lane"][vi-2:vi+3, ui-2:ui+3].any()), True)

print("\n5. a barrel 4 m ahead appears, centred, and gets bigger up close")
# ONE barrel in the scene, not all nineteen. With the 90 deg lens this test
# happened to see only barrels[0]; at the ZED's 110 deg three barrels fall in
# frame and "cone area" stopped being one silhouette, so the area went UP with
# distance and the test called a correct projection broken.
bx, by = barrels[0]
one = [(bx, by)]
areas = []
for d in (8.0, 4.0, 2.0):
    m = al.label_frame(paint, one, (bx - d, by, 0.0), 0.0, pix)
    areas.append(int(m["cone"].sum()))
    if d == 4.0:
        cols = np.nonzero(m["cone"].any(axis=0))[0]
        check("centred in u", float(cols.mean()), sg.W / 2.0, 40.0)
check("visible at 8 m", areas[0] > 0, True)
check("grows as it nears", areas[0] < areas[1] < areas[2], True)

print("\n6. nothing above the horizon is ever labelled")
m = al.label_frame(paint, barrels, (barrels[0][0] - 5, barrels[0][1], 0.0),
                   0.0, pix)
# The horizon is NOT the principal row. CY is the horizon only for a LEVEL
# camera; pitched 20 deg down it sits fy*tan(pitch) rows higher, so with the
# ZED geometry rows 108-180 are legitimately ground and this test was calling
# them sky. Derive it from the projection that draws the labels.
_x, _y, _valid = sg.pixel_to_ground()
horizon = int(np.nonzero(_valid.any(axis=1))[0][0])
check("lane above horizon", int(m["lane"][:horizon - 5].sum()), 0)
check("drivable above horizon", int(m["drivable"][:horizon - 5].sum()), 0)

print("\n7. classes do not overlap")
m = al.label_frame(paint, barrels, (bx - 4.0, by, 0.0), 0.0, pix)
check("lane n cone", int((m["lane"] & m["cone"]).sum()), 0)
check("drivable n lane", int((m["drivable"] & m["lane"]).sum()), 0)
check("drivable n cone", int((m["drivable"] & m["cone"]).sum()), 0)
check("drivable is not empty", int(m["drivable"].sum()) > 2000, True)

print("\n8. off the course, there is no paint to find")
m = al.label_frame(paint, barrels, (al.X_MIN + 1.0, al.Y_MIN + 1.0, math.pi), 0.0, pix)
check("lane pixels", int(m["lane"].sum()) < 500, True)

print("\n9. the split holds out whole blocks, never neighbouring frames")
names = [f"frame_{i:05d}" for i in range(500)]
tr, va = al.chunked_split(names, 0.2, chunk=25)
check("all frames used", len(tr) + len(va), 500)
check("no overlap", len(set(tr) & set(va)), 0)
check("val is about 20%", abs(len(va) / 500 - 0.2) < 0.06, True)
vi_ = sorted(int(n.split("_")[1]) for n in va)
runs = 1 + sum(1 for a, b in zip(vi_, vi_[1:]) if b != a + 1)
check("val is contiguous blocks, not scattered", runs <= len(va) // 20 + 1, True)

print("\n" + ("ALL CHECKS PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
