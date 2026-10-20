#!/usr/bin/env python3
"""pose_bench.py -- a fixed benchmark of camera frames at KNOWN poses, with the
car never driving.

WHY. Every accuracy number this project has produced came from a capture taken
while the robot drove itself around, and that couples the measurement to three
things that have nothing to do with the model:

  * ODOMETRY DRIFT. Poses came from integrating wheel odometry. By five minutes
    into job 109's capture the labels sat half a metre off the paint, and a
    label half a metre off turns a correct detection into a false positive. The
    measured 0.174 recall is a floor of unknown depth for that reason alone.
  * A WEDGED ROBOT. 216 of those 672 frames were byte-identical: the robot
    stopped moving and the capture kept saving the same view. A third of the
    val set was one pose.
  * WHATEVER THE RUN HAPPENED TO DO. A run that circles one corner measures the
    model on that corner. Two runs are not comparable, so neither are two
    models scored on them.

Teleporting removes all three. The pose is whatever we commanded, exact, with
no integration anywhere; nothing can freeze, because each frame is a separate
placement; and the SAME pose list produces the SAME frames every time, so model
A and model B are finally measured on identical data.

It also lets the benchmark be DESIGNED rather than observed. Poses are sampled
across the whole arena and kept only when paint is in view, so the set covers
far lines, near lines, one line, two lines, curves and the off-centre
approaches the car meets exactly when detection matters most -- cases a
well-behaved lap never visits.

WHAT THIS DOES NOT TEST, and still needs the lap runs: whether detections
arriving at 5 Hz with gaps are enough to drive on. This measures perception.
The lap benchmark measures the loop. They answer different questions and the
project needs both.

    python3 vision/pose_bench.py --plan --world igvc_oval_slow_test -n 400
    python3 vision/pose_bench.py --world igvc_oval_slow_test --out ~/esda_capture/bench
    python3 vision/pose_bench.py --selftest
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import synth_gen as sg
import autolabel

try:
    import cv2
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT = "my_robot_1"
SPAWN_Z = 0.3


# ----------------------------------------------------------------- geometry --
def yaw_to_quat(yaw):
    """(w, z) of a yaw-only rotation. x and y are zero on flat ground."""
    return math.cos(yaw / 2.0), math.sin(yaw / 2.0)


def too_close_to_barrel(x, y, barrels, keep_out):
    for bx, by in barrels:
        if (bx - x) ** 2 + (by - y) ** 2 < keep_out ** 2:
            return True
    return False


def paint_pixels_from(paint, barrels, pose, pix):
    """How many image pixels of lane paint a camera at this pose would see."""
    m = autolabel.label_frame(paint, barrels, pose, 0.0, pix)
    return int(m["lane"].sum())


def plan(world, n, seed, repo=REPO, keep_out=1.2, min_px=200, margin=2.0,
         strata=4, verbose=True):
    """Sample n poses across the arena that actually have paint in view.

    Stratified by how much paint is visible, so the set is not all easy
    frames: a benchmark made only of poses staring down a straight line
    flatters any model, and the cases that decide whether the car stays on
    course are the thin ones.
    """
    sdf = os.path.join(repo, "esda_simulation_2025", "worlds", world + ".sdf")
    tex = autolabel.ground_texture_for(sdf, repo)
    paint = autolabel.paint_mask(cv2.imread(tex, cv2.IMREAD_COLOR))
    barrels = autolabel.barrels_from_sdf(sdf)
    pix = sg.pixel_to_ground(0.0)
    rng = np.random.default_rng(seed)

    lo_x, hi_x = autolabel.X_MIN + margin, autolabel.X_MAX - margin
    lo_y, hi_y = autolabel.Y_MIN + margin, autolabel.Y_MAX - margin

    found = []
    tries = 0
    budget = max(20000, n * 150)
    while len(found) < n * 3 and tries < budget:
        tries += 1
        x = float(rng.uniform(lo_x, hi_x))
        y = float(rng.uniform(lo_y, hi_y))
        yaw = float(rng.uniform(-math.pi, math.pi))
        if too_close_to_barrel(x, y, barrels, keep_out):
            continue
        px = paint_pixels_from(paint, barrels, (x, y, yaw), pix)
        if px < min_px:
            continue
        found.append({"x": x, "y": y, "yaw": yaw, "lane_px": px})

    if not found:
        raise SystemExit(f"no poses with paint in view after {tries} tries -- "
                         f"is {world} right, and does its texture have paint?")

    # Stratify: split by visible-paint quantiles and take evenly from each
    # band, so thin views are represented instead of drowned out.
    found.sort(key=lambda p: p["lane_px"])
    out, per = [], max(1, n // strata)
    chunk = max(1, len(found) // strata)
    for s in range(strata):
        band = found[s * chunk:(s + 1) * chunk] if s < strata - 1 else found[s * chunk:]
        if not band:
            continue
        idx = np.linspace(0, len(band) - 1, min(per, len(band))).astype(int)
        out += [band[i] for i in idx]
    out = out[:n]
    rng.shuffle(out)

    if verbose:
        px = np.array([p["lane_px"] for p in out])
        print(f"[plan] {world}: {len(out)} poses from {tries} tries "
              f"({len(found)} candidates)")
        print(f"[plan] visible lane pixels: min {px.min()}  "
              f"p25 {int(np.percentile(px,25))}  median {int(np.median(px))}  "
              f"p75 {int(np.percentile(px,75))}  max {px.max()}")
        print(f"[plan] {len(barrels)} barrels, keep-out {keep_out} m")
    return out


# ----------------------------------------------------------------- teleport --
def teleport(world, x, y, yaw, z=SPAWN_Z, timeout_ms=3000):
    """Place the robot. Returns True if the service accepted it."""
    w, qz = yaw_to_quat(yaw)
    req = (f'name: "{ROBOT}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: {qz}, w: {w}}}')
    for exe in ("ign", "gz"):
        try:
            r = subprocess.run(
                [exe, "service", "-s", f"/world/{world}/set_pose",
                 "--reqtype", "ignition.msgs.Pose",
                 "--reptype", "ignition.msgs.Boolean",
                 "--timeout", str(timeout_ms), "--req", req],
                capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 5)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
        if "true" in (r.stdout or "").lower():
            return True
    return False


# ------------------------------------------------------------------ capture --
def capture(a, poses):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock

    class Grab(Node):
        def __init__(self):
            super().__init__("pose_bench")
            self.img = None
            self.stamp = None
            self.sim_t = None
            self.create_subscription(Image, a.image_topic, self._img,
                                     qos_profile_sensor_data)
            self.create_subscription(Clock, "/clock", self._clk, 10)

        def _img(self, m):
            h, w = m.height, m.width
            buf = np.frombuffer(m.data, np.uint8)
            if m.encoding in ("rgb8", "bgr8"):
                img = buf.reshape(h, w, 3)
                if m.encoding == "rgb8":
                    img = img[:, :, ::-1]
            else:
                return
            self.img = img.copy()
            self.stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        def _clk(self, m):
            self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = Grab()

    # Wait for the first frame so we know the stack is actually up.
    t0 = time.time()
    while rclpy.ok() and n.img is None and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.img is None:
        raise SystemExit(f"no frames on {a.image_topic} after 60 s -- is the "
                         f"sim up? (./demo.sh up --no-rviz --world {a.world})")

    saved = failed = 0
    for i, p in enumerate(poses):
        if not teleport(a.world, p["x"], p["y"], p["yaw"]):
            failed += 1
            if failed <= 3:
                print(f"[bench] teleport refused for pose {i}")
            if failed > 10 and saved == 0:
                raise SystemExit("teleport is not working -- check the world "
                                 "name and that the model is called " + ROBOT)
            continue

        # Let the physics settle and, critically, wait for a frame RENDERED
        # after the move. Reusing the frame already in hand would label the
        # new pose with the old view -- silently, and in a way that looks
        # exactly like a model that cannot see.
        target = (n.sim_t or 0.0) + a.settle
        n.img = None
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < a.frame_timeout:
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.img is not None and (n.sim_t or 0.0) >= target:
                break
        if n.img is None:
            failed += 1
            continue

        stem = f"frame_{saved:05d}"
        cv2.imwrite(os.path.join(a.out, stem + ".jpg"), n.img)
        with open(os.path.join(a.out, stem + ".json"), "w") as fh:
            json.dump({"x": p["x"], "y": p["y"], "yaw": p["yaw"],
                       "world": a.world, "t": n.sim_t,
                       "lane_px_expected": p.get("lane_px"),
                       "pose_source": "commanded"}, fh)
        saved += 1
        if saved % 25 == 0:
            print(f"[bench] {saved}/{len(poses)}")

    n.destroy_node()
    rclpy.shutdown()
    print(f"[bench] saved {saved} frames to {a.out} ({failed} failed)")
    if saved < len(poses) * 0.8:
        raise SystemExit(f"only {saved} of {len(poses)} poses captured")
    return saved


# ----------------------------------------------------------------- selftest --
def selftest():
    print("=" * 72)
    print("pose_bench self-test -- geometry only, no sim needed")
    print("=" * 72)
    fails = []

    def ck(name, got, want, tol=1e-9):
        ok = (abs(got - want) <= tol) if isinstance(want, float) else (got == want)
        print("  %-52s %s" % (name, "ok" if ok else f"FAIL got {got!r} want {want!r}"))
        if not ok:
            fails.append(name)

    w, z = yaw_to_quat(0.0)
    ck("yaw 0 -> w", w, 1.0)
    ck("yaw 0 -> z", z, 0.0)
    w, z = yaw_to_quat(math.pi)
    ck("yaw pi -> w", w, 0.0, 1e-9)
    ck("yaw pi -> z", z, 1.0, 1e-9)
    w, z = yaw_to_quat(math.pi / 2)
    ck("yaw pi/2 -> w", w, math.sqrt(0.5), 1e-12)
    ck("yaw pi/2 -> z", z, math.sqrt(0.5), 1e-12)
    for y in (-3.0, -1.263, 0.0, 0.5, 1.263, 3.0):
        w, z = yaw_to_quat(y)
        ck(f"quat {y:+.3f} is unit", w * w + z * z, 1.0, 1e-12)
        ck(f"quat {y:+.3f} round-trips", 2 * math.atan2(z, w), y, 1e-12)

    bs = [(0.0, 0.0), (5.0, 5.0)]
    ck("on top of a barrel is too close", too_close_to_barrel(0.1, 0.0, bs, 1.2), True)
    ck("1.3 m away is not", too_close_to_barrel(1.3, 0.0, bs, 1.2), False)
    ck("just inside the keep-out", too_close_to_barrel(1.19, 0.0, bs, 1.2), True)
    ck("far from both", too_close_to_barrel(-9.0, 2.0, bs, 1.2), False)
    ck("empty barrel list", too_close_to_barrel(0.0, 0.0, [], 1.2), False)

    print()
    if fails:
        print(f"{len(fails)} FAILED: " + ", ".join(fails))
        raise SystemExit(1)
    print("ALL CHECKS PASSED")
    print("=" * 72)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--world", default="igvc_oval_slow_test")
    p.add_argument("--out", default=os.path.expanduser("~/esda_capture/bench"))
    p.add_argument("--poses", default="", help="reuse a poses.json instead of sampling")
    p.add_argument("--plan", action="store_true", help="sample and write poses.json, do not capture")
    p.add_argument("-n", "--n-poses", type=int, default=400)
    p.add_argument("--seed", type=int, default=11)
    p.add_argument("--min-px", type=int, default=200,
                   help="reject a pose with less visible paint than this")
    p.add_argument("--keep-out", type=float, default=1.2, help="metres clear of a barrel")
    p.add_argument("--settle", type=float, default=0.4, help="simulated seconds after a teleport")
    p.add_argument("--frame-timeout", type=float, default=20.0, help="wall seconds")
    p.add_argument("--image-topic", default="/head_camera/images")
    p.add_argument("--repo", default=REPO)
    a = p.parse_args()

    if a.selftest:
        return selftest()
    if not HAVE_CV:
        raise SystemExit("needs opencv")

    if a.poses:
        poses = json.load(open(a.poses))
        print(f"[bench] {len(poses)} poses from {a.poses}")
    else:
        poses = plan(a.world, a.n_poses, a.seed, a.repo,
                     keep_out=a.keep_out, min_px=a.min_px)
        os.makedirs(a.out, exist_ok=True)
        pj = os.path.join(a.out, "poses.json")
        json.dump(poses, open(pj, "w"))
        print(f"[bench] wrote {pj}")

    if a.plan:
        return 0
    return 0 if capture(a, poses) else 1


if __name__ == "__main__":
    raise SystemExit(main())
