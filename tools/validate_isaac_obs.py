#!/usr/bin/env python3
"""Offline verification of the Isaac RL observation and action contract.

WHAT THIS IS
    The counterpart to tools/validate_waypoint_logic.py, for the Isaac work.
    It needs no ROS, no Isaac Sim, no GPU and no Gazebo -- numpy and pyyaml
    only -- so it runs in WSL, on Windows, or in CI, and it is the gate a
    checkpoint has to pass before it is worth evaluating.

    It follows the same principle as that harness: compare the fast
    implementation against an independent brute-force one, so the question
    answered is "did the fast version change any answer", not merely "does it
    run".  scan_sim.brute_force_scan is that oracle -- it marches each ray in
    1 cm steps and tests as it goes, which shares no algebra with the
    vectorised ray/disc and ray/segment solvers.

    The constants lint is the other half.  obs_core.py hard-codes a wheel
    radius, a wheel separation and a velocity envelope that are really owned by
    description/robot_core_ref.xacro, config/my_controllers_2wd.yaml and
    config/nav2_params.yaml.  Nothing stops those drifting apart, and a policy
    trained against a stale wheel radius produces confident, wrong velocities
    rather than an error.  This is the same class of bug as the
    observation_sources mismatch that silently disabled two costmap layers.

USAGE
    python3 tools/validate_isaac_obs.py
    python3 tools/validate_isaac_obs.py --quick     # skip the slow oracle suites
"""
import argparse
import math
import os
import sys
import xml.dom.minidom as minidom

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISAAC_DIR = os.path.join(REPO, "research", "isaac")
PKG = os.path.join(REPO, "src", "esda_simulation_2025")
sys.path.insert(0, ISAAC_DIR)

import obs_core as OC            # noqa: E402
import scan_sim as SS            # noqa: E402

RESULTS = []


def check(name, condition, detail=""):
    ok = bool(condition)
    RESULTS.append((name, ok, detail))
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name,
                           ("   " + detail) if detail and not ok else ""))
    return ok


def close(a, b, tol=1e-9):
    return abs(float(a) - float(b)) <= tol


# ====================================================== suite 1: constants --
def _xacro_properties(path):
    doc = minidom.parse(path)
    out = {}
    for el in doc.getElementsByTagName("xacro:property"):
        name = el.getAttribute("name")
        try:
            out[name] = float(el.getAttribute("value"))
        except ValueError:
            out[name] = el.getAttribute("value")
    return out


def _urdf_wheel_geometry(path):
    doc = minidom.parse(path)
    radius, ys = None, {}
    for link in doc.getElementsByTagName("link"):
        if link.getAttribute("name") != "left_wheel":
            continue
        for cyl in link.getElementsByTagName("cylinder"):
            radius = float(cyl.getAttribute("radius"))
            break
    for joint in doc.getElementsByTagName("joint"):
        n = joint.getAttribute("name")
        if n not in ("left_wheel_joint", "right_wheel_joint"):
            continue
        for origin in joint.getElementsByTagName("origin"):
            ys[n] = float(origin.getAttribute("xyz").split()[1])
            break
    return radius, ys


def suite_constants():
    print("\n[1] constants lint -- obs_core vs the files that actually own them")
    try:
        import yaml
    except ImportError:
        check("pyyaml available", False, "pip install pyyaml")
        return

    ctrl_path = os.path.join(PKG, "config", "my_controllers_2wd.yaml")
    nav_path = os.path.join(PKG, "config", "nav2_params.yaml")
    xacro_path = os.path.join(PKG, "description", "robot_core_ref.xacro")
    urdf_path = os.path.join(ISAAC_DIR, "assets", "esda_robot.urdf")

    with open(ctrl_path) as fh:
        ctrl = yaml.safe_load(fh)["diff_drive_base_controller"]["ros__parameters"]

    check("wheel_radius: obs_core == my_controllers_2wd.yaml",
          close(OC.WHEEL_RADIUS, ctrl["wheel_radius"]),
          "obs_core %r vs yaml %r" % (OC.WHEEL_RADIUS, ctrl["wheel_radius"]))
    check("wheel_separation: obs_core == my_controllers_2wd.yaml",
          close(OC.WHEEL_SEPARATION, ctrl["wheel_separation"]),
          "obs_core %r vs yaml %r" % (OC.WHEEL_SEPARATION, ctrl["wheel_separation"]))
    check("a_lin: obs_core == controller linear.x.max_acceleration",
          close(OC.A_LIN, ctrl["linear.x.max_acceleration"]))
    check("a_ang: obs_core == controller angular.z.max_acceleration",
          close(OC.A_ANG, ctrl["angular.z.max_acceleration"]))

    props = _xacro_properties(xacro_path)
    check("wheel_radius: obs_core == robot_core_ref.xacro",
          close(OC.WHEEL_RADIUS, props.get("wheel_radius", -1)),
          "xacro %r" % props.get("wheel_radius"))
    check("wheel_separation == 2 * wheel_offset_y in robot_core_ref.xacro",
          close(OC.WHEEL_SEPARATION, 2.0 * props.get("wheel_offset_y", -1)),
          "xacro offset %r" % props.get("wheel_offset_y"))

    if os.path.exists(urdf_path):
        radius, ys = _urdf_wheel_geometry(urdf_path)
        check("exported URDF wheel radius matches obs_core",
              radius is not None and close(OC.WHEEL_RADIUS, radius),
              "urdf %r" % radius)
        sep = abs(ys.get("left_wheel_joint", 0.0) - ys.get("right_wheel_joint", 0.0))
        check("exported URDF wheel separation matches obs_core",
              close(OC.WHEEL_SEPARATION, sep), "urdf %r" % sep)
    else:
        check("exported URDF present", False,
              "run tools/export_urdf_for_isaac.py first")

    with open(nav_path) as fh:
        nav = yaml.safe_load(fh)
    mppi = nav["controller_server"]["ros__parameters"]["FollowPath"]
    check("v_max: obs_core == nav2 MPPI vx_max",
          close(OC.V_MAX, mppi["vx_max"]),
          "obs_core %r vs nav2 %r" % (OC.V_MAX, mppi["vx_max"]))
    check("w_max: obs_core == nav2 MPPI wz_max",
          close(OC.W_MAX, mppi["wz_max"]),
          "obs_core %r vs nav2 %r" % (OC.W_MAX, mppi["wz_max"]))
    check("control_hz: obs_core == nav2 controller_frequency",
          close(OC.CONTROL_HZ,
                nav["controller_server"]["ros__parameters"]["controller_frequency"]))
    check("robot_radius: obs_core == nav2 local costmap",
          close(OC.ROBOT_RADIUS,
                nav["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"]))

    # The command interface in ros2_control_2wd.xacro caps wheel velocity at
    # +/-10 rad/s, which is a tighter bound than the controller's 2.0 m/s.
    wl, wr = OC.twist_to_wheels(OC.V_MAX, OC.W_MAX)
    check("V_MAX + W_MAX stays inside the +/-10 rad/s command interface",
          max(abs(wl), abs(wr)) <= 10.0,
          "worst wheel %.3f rad/s" % max(abs(wl), abs(wr)))


# ================================================== suite 2: scan geometry --
def suite_scan_geometry():
    print("\n[2] scan geometry matches description/lidar.xacro")
    ang = OC.beam_angles()
    check("898 beams", ang.shape == (898,))
    check("angle_min", close(ang[0], -3.14159))
    check("angle_max", close(ang[-1], 3.13741))
    inc = ang[1] - ang[0]
    check("angle_increment ~ 0.007 rad (velodyne_laserscan resolution)",
          abs(inc - 0.007) < 2e-5, "got %.6f" % inc)
    check("trim leaves an exact pooling factor",
          (OC.N_BEAMS_RAW - OC.TRIM_BEAMS) == OC.N_POOLED * OC.BEAMS_PER_BUCKET,
          "%d vs %d*%d" % (OC.N_BEAMS_RAW - OC.TRIM_BEAMS,
                           OC.N_POOLED, OC.BEAMS_PER_BUCKET))
    check("trimmed arc is rear-facing and tiny",
          abs(ang[-1] - ang[-1 - OC.TRIM_BEAMS]) < 0.02 and ang[-1] > 3.0)


# ======================================================== suite 3: pooling --
def suite_pooling():
    print("\n[3] beam pooling")
    rng = np.random.default_rng(1)
    raw = rng.uniform(0.2, 9.0, size=OC.N_BEAMS_RAW)
    pooled = OC.pool_scan(raw)
    check("pooled length", pooled.shape == (OC.N_POOLED,))

    manual = raw[:OC.N_BEAMS_RAW - OC.TRIM_BEAMS].reshape(
        OC.N_POOLED, OC.BEAMS_PER_BUCKET).min(axis=1)
    check("pooling is the per-bucket minimum",
          np.allclose(pooled, manual))

    # A single near return must survive pooling: that is the whole reason for
    # min rather than mean.
    spiky = np.full(OC.N_BEAMS_RAW, 9.0)
    spiky[3] = 0.4
    check("a lone near return survives pooling",
          close(OC.pool_scan(spiky)[0], 0.4))
    check("a mean would have hidden it",
          spiky[:OC.BEAMS_PER_BUCKET].mean() > 1.0)

    inf_scan = np.full(OC.N_BEAMS_RAW, np.inf)
    inf_scan[0] = 2.0
    out = OC.pool_scan(inf_scan)
    check("inf becomes max range, never NaN",
          np.all(np.isfinite(out)) and close(out[0], 2.0)
          and close(out[1], OC.RANGE_CLIP))

    nan_scan = np.full(OC.N_BEAMS_RAW, np.nan)
    check("all-NaN scan yields finite output",
          np.all(np.isfinite(OC.pool_scan(nan_scan))))

    try:
        OC.pool_scan(np.ones(900))
        wrong_count_raises = False
    except ValueError:
        wrong_count_raises = True
    check("a wrong beam count raises instead of silently resampling",
          wrong_count_raises)

    odd = rng.uniform(0.2, 9.0, size=721)
    a = OC.resample_scan(odd, OC.N_POOLED)
    b = OC.resample_scan(odd.copy(), OC.N_POOLED)
    check("resample_scan is deterministic", np.array_equal(a, b))
    check("resample_scan handles a non-divisor count", a.shape == (OC.N_POOLED,))
    check("resample_scan never exceeds the true minimum",
          a.min() >= odd.min() - 1e-12)


# ================================================ suite 4: ray primitives --
def suite_ray_primitives():
    print("\n[4] ray/disc and ray/segment against hand-computed answers")
    cfg = SS.ScanCfg(n_beams=4, angle_min=0.0, angle_max=3.0 * math.pi / 2.0)
    dirs = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])
    origin = np.array([0.0, 0.0])

    d = SS.ray_discs(origin, dirs, [[5.0, 0.0]], [1.0], 0.1, 100.0)
    check("disc straight ahead at 5 m, radius 1 -> 4 m", close(d[0], 4.0, 1e-9))
    check("no hit on the other three beams", np.all(np.isinf(d[1:])))

    d = SS.ray_discs(origin, dirs, [[5.0, 0.0]], [1.0], 0.1, 3.0)
    check("beyond range_max is a miss", np.all(np.isinf(d)))

    d = SS.ray_discs(origin, dirs, [[0.0, 0.0]], [2.0], 0.1, 100.0)
    check("sensor inside a disc returns the exit point", close(d[0], 2.0, 1e-9))

    # A segment crossing x = 3, spanning y in [-1, 1].
    s = SS.ray_segments(origin, dirs, [[3.0, -1.0]], [[3.0, 1.0]], 0.1, 100.0)
    check("segment crossing ahead at 3 m", close(s[0], 3.0, 1e-9))
    check("segment missed by the perpendicular beams", np.all(np.isinf(s[1:])))

    # Parallel: a segment lying along the +x axis must not register a hit.
    s = SS.ray_segments(origin, dirs, [[1.0, 0.0]], [[5.0, 0.0]], 0.1, 100.0)
    check("a parallel collinear segment does not produce a spurious hit",
          np.isinf(s[0]) or s[0] >= 1.0)

    # u outside [0,1]: the segment's span does not reach the ray.
    s = SS.ray_segments(origin, dirs, [[3.0, 2.0]], [[3.0, 5.0]], 0.1, 100.0)
    check("a segment that does not span the ray is a miss", np.isinf(s[0]))

    p0, p1, keep = SS.cull_segments(origin, [[50.0, 0.0], [1.0, 0.0]],
                                    [[51.0, 0.0], [2.0, 0.0]], 10.0)
    check("culling keeps the near segment and drops the far one",
          p0.shape[0] == 1 and close(p0[0][0], 1.0))

    # Exactness, without reference to the oracle: a reported range must put the
    # hit point ON the surface it claims to have hit. This is the tight check;
    # the brute-force oracle in suite 5 is only accurate to its 1 cm step.
    rng = np.random.default_rng(21)
    full = SS.ScanCfg(n_beams=898, range_max=50.0)
    ang = full.angles()
    pose = (0.4, -0.7, 0.9)
    D = np.stack([np.cos(ang + pose[2]), np.sin(ang + pose[2])], axis=1)
    O = np.array(pose[:2])

    centres = rng.uniform(-15, 15, size=(12, 2))
    radii = rng.uniform(0.25, 0.8, size=12)
    r = SS.synth_scan(pose, full, centres, radii, None)
    fin = np.isfinite(r)
    pts = O[None, :] + r[fin, None] * D[fin]
    resid = (np.linalg.norm(pts[:, None, :] - centres[None, :, :], axis=2)
             - radii[None, :]).min(axis=1)
    check("every disc hit point lies exactly on a disc surface",
          fin.sum() > 50 and float(np.abs(resid).max()) < 1e-9,
          "%d hits, worst residual %.2e" % (int(fin.sum()),
                                            float(np.abs(resid).max()) if fin.any() else 0))

    poly = np.array([[-9.0, 3.0], [0.0, 3.4], [9.0, 2.6]])
    r = SS.synth_scan(pose, full, None, None, [poly])
    fin = np.isfinite(r)
    pts = O[None, :] + r[fin, None] * D[fin]
    a, b = SS.polyline_segments(poly, closed=False)
    worst = 0.0
    for px, py in pts:
        e = b - a
        L2 = np.maximum((e * e).sum(axis=1), 1e-30)
        u = np.clip(((px - a[:, 0]) * e[:, 0] + (py - a[:, 1]) * e[:, 1]) / L2, 0, 1)
        d = np.hypot(a[:, 0] + u * e[:, 0] - px, a[:, 1] + u * e[:, 1] - py).min()
        worst = max(worst, d)
    check("every lane hit point lies exactly on a boundary segment",
          fin.sum() > 20 and worst < 1e-9,
          "%d hits, worst %.2e" % (int(fin.sum()), worst))


# =========================================== suite 5: vectorised vs oracle --
def suite_oracle(n_cases=6):
    print("\n[5] synth_scan vs the brute-force oracle (independent implementation)")
    rng = np.random.default_rng(7)
    # Few beams: the oracle marches every ray in 1 cm steps and is very slow.
    cfg = SS.ScanCfg(n_beams=64, range_max=20.0)
    worst = 0.0
    for case in range(n_cases):
        pose = (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi))
        k = rng.integers(2, 6)
        discs = rng.uniform(-9, 9, size=(k, 2))
        radii = rng.uniform(0.25, 0.6, size=k)
        poly = np.array([[-8.0, 4.0], [0.0, 4.5], [8.0, 3.5]])

        fast = SS.synth_scan(pose, cfg, discs, radii, [poly])
        slow = SS.brute_force_scan(pose, cfg, discs, radii, [poly], step=0.01)

        both = np.isfinite(fast) & np.isfinite(slow)
        agree = np.isfinite(fast) == np.isfinite(slow)
        # The oracle reports the first sample INSIDE the obstacle, so it reads
        # up to one step long; and within one step of a tangent the two can
        # legitimately disagree about whether a ray grazes at all.
        err = np.abs(fast[both] - slow[both]).max() if both.any() else 0.0
        worst = max(worst, err)
        mismatch = int((~agree).sum())
        check("case %d: hit/miss agreement (<=2 grazing beams)" % case,
              mismatch <= 2, "%d beams disagree" % mismatch)
    # The oracle is the approximate one here, and it errs in BOTH directions:
    # for a disc it reports the first sample strictly inside, so it reads up to
    # one step long; for a segment it fires as soon as a sample is within `step`
    # of the line, so it reads up to one step short. A ray that grazes a barrel
    # and a lane line in quick succession can therefore differ by ~1.5 steps.
    # The exactness check in suite 4 is the tight one -- it pins hit points to
    # the surface at 1e-9 without consulting the oracle at all.
    check("worst range disagreement within the oracle's two-sided step bound",
          worst <= 0.015, "worst %.4f m (step 0.01)" % worst)


# ================================================ suite 6: the FOV asymmetry --
def suite_fov_asymmetry():
    print("\n[6] lane paint is camera-limited, obstacles are not")
    cfg = SS.ScanCfg()
    pose = (0.0, 0.0, 0.0)

    behind = np.array([[-2.0, -5.0], [-2.0, 5.0]])     # paint 2 m behind
    r = SS.synth_scan(pose, cfg, None, None, [behind])
    check("paint behind the robot produces NO return",
          np.all(np.isinf(r)),
          "%d beams returned" % int(np.isfinite(r).sum()))

    r = SS.synth_scan(pose, cfg, [[-2.0, 0.0]], [0.3], None)
    check("an obstacle behind the robot DOES return",
          np.isfinite(r).any())

    ahead = np.array([[3.0, -5.0], [3.0, 5.0]])
    r = SS.synth_scan(pose, cfg, None, None, [ahead])
    hit = np.isfinite(r)
    ang = cfg.angles()
    check("paint ahead returns", hit.any())
    check("every lane return lies inside the camera cone",
          bool(np.all(np.abs(ang[hit]) <= cfg.lane_half_fov + 1e-9)),
          "widest %.3f rad vs half-FOV %.3f" % (
              float(np.abs(ang[hit]).max()) if hit.any() else 0.0, cfg.lane_half_fov))

    far = np.array([[20.0, -5.0], [20.0, 5.0]])
    r = SS.synth_scan(pose, cfg, None, None, [far])
    check("paint beyond the depth clip produces no return", np.all(np.isinf(r)))


# ===================================================== suite 7: kinematics --
def suite_kinematics():
    print("\n[7] differential drive round trip")
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(500):
        v = rng.uniform(OC.V_MIN, OC.V_MAX)
        w = rng.uniform(-OC.W_MAX, OC.W_MAX)
        wl, wr = OC.twist_to_wheels(v, w)
        back = OC.wheels_to_twist(wl, wr)
        worst = max(worst, abs(back[0] - v), abs(back[1] - w))
    check("twist -> wheels -> twist is identity", worst < 1e-12,
          "worst %.3e" % worst)

    wl, wr = OC.twist_to_wheels(1.0, 0.0)
    check("1 m/s straight -> both wheels 1/0.1625 rad/s",
          close(wl, 1.0 / 0.1625, 1e-9) and close(wr, wl, 1e-12))

    wl, wr = OC.twist_to_wheels(0.0, 1.0)
    check("spin in place -> equal and opposite wheels",
          close(wl, -wr, 1e-12) and wr > 0)


# ======================================================== suite 8: actions --
def suite_actions():
    print("\n[8] action mapping and rate limiting")
    check("action -1 -> V_MIN", close(OC.action_to_twist([-1, 0])[0], OC.V_MIN))
    check("action +1 -> V_MAX", close(OC.action_to_twist([1, 0])[0], OC.V_MAX))
    check("action 0 on yaw -> 0 rad/s", close(OC.action_to_twist([0, 0])[1], 0.0))
    check("action +1 on yaw -> W_MAX", close(OC.action_to_twist([0, 1])[1], OC.W_MAX))

    out = OC.action_to_twist([5.0, -5.0])
    check("out-of-range actions are clipped, not extrapolated",
          close(out[0], OC.V_MAX) and close(out[1], -OC.W_MAX))

    rng = np.random.default_rng(11)
    worst_dv = worst_dw = 0.0
    prev = np.array([0.0, 0.0])
    for _ in range(200):
        a = rng.uniform(-1, 1, size=2)
        nxt = OC.action_to_twist(a, prev)
        worst_dv = max(worst_dv, abs(nxt[0] - prev[0]))
        worst_dw = max(worst_dw, abs(nxt[1] - prev[1]))
        prev = nxt
    check("linear acceleration never exceeds A_LIN*dt",
          worst_dv <= OC.A_LIN * OC.CONTROL_DT + 1e-12,
          "worst %.4f vs limit %.4f" % (worst_dv, OC.A_LIN * OC.CONTROL_DT))
    check("angular acceleration never exceeds A_ANG*dt",
          worst_dw <= OC.A_ANG * OC.CONTROL_DT + 1e-12,
          "worst %.4f vs limit %.4f" % (worst_dw, OC.A_ANG * OC.CONTROL_DT))

    worst = 0.0
    for _ in range(200):
        a = rng.uniform(-1, 1, size=2)
        back = OC.twist_to_action(OC.action_to_twist(a))
        worst = max(worst, float(np.abs(back - a).max()))
    check("unlimited action -> twist -> action is identity", worst < 1e-12,
          "worst %.3e" % worst)


# =================================================== suite 9: observations --
def suite_observation():
    print("\n[9] observation assembly")
    rng = np.random.default_rng(5)
    raw = np.full(OC.N_BEAMS_RAW, np.inf)
    raw[::9] = rng.uniform(0.3, 8.0, size=raw[::9].shape)
    pooled = OC.pool_scan(raw)

    frame = OC.encode_frame(pooled, (0.5, -0.2), (1.0, 2.0), 0.3, (4.0, 6.0), (0.1, -0.2))
    check("frame length", frame.shape == (OC.OBS_PER_FRAME,))
    check("frame is finite", bool(np.all(np.isfinite(frame))))
    check("scan block is normalised to [0,1]",
          bool(np.all(frame[:OC.N_POOLED] >= 0.0) and np.all(frame[:OC.N_POOLED] <= 1.0)))
    check("velocity block normalised by the limits",
          close(frame[OC.N_POOLED], 0.5 / OC.V_MAX)
          and close(frame[OC.N_POOLED + 1], -0.2 / OC.W_MAX))

    c, s = frame[OC.N_POOLED + 2], frame[OC.N_POOLED + 3]
    check("goal bearing is a unit vector", close(c * c + s * s, 1.0, 1e-12))
    check("goal distance normalised and clipped",
          0.0 <= frame[OC.N_POOLED + 4] <= 1.0)

    far = OC.goal_features((0, 0), 0.0, (1000.0, 0.0))
    check("a very distant goal saturates at 1.0", close(far[2], 1.0))

    # Bearing continuity across the +/-pi seam, which is exactly where a robot
    # that must turn around sits.
    a = OC.goal_features((0, 0), 0.0, (-1.0, 1e-9))
    b = OC.goal_features((0, 0), 0.0, (-1.0, -1e-9))
    check("goal encoding is continuous across +/-pi",
          float(np.abs(a[:2] - b[:2]).max()) < 1e-6)

    obs = OC.stack_history([frame] * OC.N_HISTORY)
    check("stacked observation length", obs.shape == (OC.OBS_DIM,))
    check("OBS_DIM is consistent",
          OC.OBS_DIM == OC.OBS_PER_FRAME * OC.N_HISTORY)

    try:
        OC.stack_history([frame])
        raised = False
    except ValueError:
        raised = True
    check("a short history raises", raised)


# ====================================================== suite 10: the meta --
def suite_meta():
    print("\n[10] deployment contract")
    meta = OC.policy_meta()
    check("a fresh meta agrees with the code", OC.check_meta(meta) == [])
    check("meta records the full observation layout",
          meta["obs_dim"] == OC.OBS_DIM and meta["action_dim"] == OC.ACTION_DIM)

    bad = dict(meta)
    bad["n_pooled"] = meta["n_pooled"] + 8
    problems = OC.check_meta(bad)
    check("a pooling-width mismatch is caught",
          any("n_pooled" in p for p in problems), str(problems))

    bad = dict(meta)
    bad["w_max"] = 1.2
    check("a changed velocity envelope is caught",
          any("w_max" in p for p in OC.check_meta(bad)))

    bad = dict(meta)
    del bad["wheel_radius"]
    check("a missing key is caught",
          any("wheel_radius" in p for p in OC.check_meta(bad)))


# ============================================= suite 11: course boundaries --
def suite_boundaries():
    print("\n[11] lane boundaries from a centreline")
    t = np.linspace(0, 2 * math.pi, 200, endpoint=False)
    centre = np.stack([10.0 * np.cos(t), 10.0 * np.sin(t)], axis=1)
    widths = np.full(200, 3.0)
    left, right = SS.boundaries_from_centreline(centre, widths)

    rl = np.linalg.norm(left, axis=1)
    rr = np.linalg.norm(right, axis=1)
    # t increases, so this circle is traversed counter-clockwise, and "left"
    # means left OF THE DIRECTION OF TRAVEL -- which on a counter-clockwise
    # loop is the inside. Hence left = 10 - 1.5 and right = 10 + 1.5. Worth
    # pinning down: swapping these mirrors every lane boundary in training,
    # and the policy would learn a course that is inside out.
    check("counter-clockwise loop: left boundary is the INNER one",
          np.allclose(rl, 8.5, atol=1e-3) and np.allclose(rr, 11.5, atol=1e-3),
          "left %.4f right %.4f" % (rl.mean(), rr.mean()))

    rev = centre[::-1].copy()
    lrev, rrev = SS.boundaries_from_centreline(rev, widths)
    check("reversing the course swaps the two boundaries",
          np.allclose(np.linalg.norm(lrev, axis=1), 11.5, atol=1e-3)
          and np.allclose(np.linalg.norm(rrev, axis=1), 8.5, atol=1e-3))
    check("boundary separation equals the width everywhere",
          np.allclose(np.linalg.norm(left - right, axis=1), 3.0, atol=1e-3))

    try:
        SS.boundaries_from_centreline(centre, widths[:-1])
        raised = False
    except ValueError:
        raised = True
    check("mismatched centreline and width lengths raise", raised)


# ============================================== suite 12: the real course --
def suite_course_meta():
    print("\n[12] research/isaac/config/course_meta.yaml")
    path = os.path.join(ISAAC_DIR, "config", "course_meta.yaml")
    if not os.path.exists(path):
        check("course_meta.yaml present", False,
              "run research/isaac/scripts/extract_centreline.py")
        return
    try:
        import yaml
    except ImportError:
        check("pyyaml available", False)
        return
    with open(path) as fh:
        meta = yaml.safe_load(fh)

    rows = np.array(meta["centreline"], dtype=np.float64)
    check("centreline is (N,3)", rows.ndim == 2 and rows.shape[1] == 3)
    pts, widths = rows[:, :2], rows[:, 2]

    check("closed loop flag", meta.get("closed") is True)
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    check("samples are evenly spaced", float(seg.std()) < 0.05,
          "std %.4f m" % float(seg.std()))
    check("lap length agrees with the samples",
          abs(seg.sum() - meta["lap_length_m"]) < 1.0,
          "sum %.2f vs recorded %.2f" % (seg.sum(), meta["lap_length_m"]))

    # The course quad is 43 x 37 m placed at (0, 16.3); see igvc.sdf.
    check("every sample lies inside the arena",
          bool(np.all(np.abs(pts[:, 0]) <= 21.5)
               and np.all(pts[:, 1] >= -2.2) and np.all(pts[:, 1] <= 34.8)),
          "x [%.1f %.1f] y [%.1f %.1f]" % (pts[:, 0].min(), pts[:, 0].max(),
                                           pts[:, 1].min(), pts[:, 1].max()))
    check("corridor is always wider than the robot",
          float(widths.min()) > 2.0 * OC.ROBOT_RADIUS,
          "min width %.2f m vs 2*robot_radius %.2f" % (widths.min(), 2 * OC.ROBOT_RADIUS))
    check("median corridor width agrees with lane_half_width 1.5",
          abs(float(np.median(widths)) - 3.0) < 0.5,
          "median %.2f m" % float(np.median(widths)))
    check("the world/pixel check recorded by the extractor was sane",
          float(meta.get("world_pixel_check_worst_err_m", 99)) < 1.0,
          "worst %.2f m" % float(meta.get("world_pixel_check_worst_err_m", 99)))

    # And the whole point: a scan taken on the centreline should see the
    # boundaries at roughly half the corridor width.
    left, right = SS.boundaries_from_centreline(pts, widths)
    cfg = SS.ScanCfg()
    i = len(pts) // 4
    heading = math.atan2(pts[(i + 1) % len(pts), 1] - pts[i, 1],
                         pts[(i + 1) % len(pts), 0] - pts[i, 0])
    r = SS.synth_scan((pts[i, 0], pts[i, 1], heading), cfg, None, None, [left, right])
    finite = r[np.isfinite(r)]
    check("standing on the centreline, paint is visible",
          finite.size > 0, "no lane returns at sample %d" % i)
    if finite.size:
        # NOT half the corridor width.  The paint directly beside the robot
        # sits at +/-90 degrees, far outside the camera's 1.089 rad cone, so
        # the nearest paint the robot can actually see is where the cone edge
        # meets the boundary: half_width / sin(half_fov), about 1.9x further.
        #
        # This is a real property of the sensor suite, not a modelling
        # shortcut, and it is a large part of why lane keeping is hard on this
        # robot: it is effectively blind to the line it is about to cross.
        expect = (widths[i] / 2.0) / math.sin(cfg.lane_half_fov)
        check("nearest visible paint matches half_width / sin(half_fov)",
              abs(float(finite.min()) - expect) < 0.6,
              "nearest %.2f m, predicted %.2f m (half-width %.2f)"
              % (finite.min(), expect, widths[i] / 2.0))
        check("the robot cannot see the paint directly beside it",
              float(finite.min()) > widths[i] / 2.0 * 1.4,
              "nearest %.2f m vs half-width %.2f" % (finite.min(), widths[i] / 2.0))


# ========================================================= suite 13: torch --
def suite_torch():
    print("\n[13] torch twin vs the numpy reference")
    try:
        import torch
    except ImportError:
        print("  [SKIP] torch not importable here (expected in WSL/ROS)")
        return
    try:
        import scan_sim_torch as ST
    except ImportError:
        print("  [SKIP] scan_sim_torch.py not written yet")
        return

    rng = np.random.default_rng(13)
    cfg = SS.ScanCfg(n_beams=256, range_max=30.0)
    poses, discs, radii = [], [], []
    for _ in range(8):
        poses.append((rng.uniform(-5, 5), rng.uniform(-5, 5),
                      rng.uniform(-math.pi, math.pi)))
    discs = rng.uniform(-12, 12, size=(10, 2))
    radii = rng.uniform(0.25, 0.6, size=10)
    poly = np.array([[-12.0, 6.0], [0.0, 6.5], [12.0, 5.5]])

    ref = np.stack([SS.synth_scan(p, cfg, discs, radii, [poly]) for p in poses])
    got = ST.synth_scan_batch(np.array(poses), cfg, discs, radii, [poly])
    got = np.asarray(got)

    check("batched shape matches", got.shape == ref.shape, str(got.shape))
    agree = np.isfinite(ref) == np.isfinite(got)
    check("hit/miss pattern identical", bool(np.all(agree)),
          "%d beams differ" % int((~agree).sum()))
    both = np.isfinite(ref) & np.isfinite(got)
    err = float(np.abs(ref[both] - got[both]).max()) if both.any() else 0.0
    check("ranges agree to float32 precision", err < 1e-3, "worst %.3e" % err)

    # The pooling rule is the one piece of maths that must match the DEPLOYED
    # node exactly, not just closely: it is applied on both sides of the
    # sim-to-real boundary. Compare against obs_core, not against itself.
    rng2 = np.random.default_rng(17)
    raw = rng2.uniform(0.2, 9.5, size=(5, OC.N_BEAMS_RAW))
    raw[0, ::13] = np.inf
    raw[1, 40] = 0.11
    ref_pool = np.stack([OC.pool_scan(r) for r in raw])
    got_pool = np.asarray(ST.pool_scan_batch(
        torch.as_tensor(raw, dtype=torch.float64)))
    check("pool_scan_batch reproduces obs_core.pool_scan exactly",
          got_pool.shape == ref_pool.shape
          and float(np.abs(got_pool - ref_pool).max()) < 1e-12,
          "worst %.3e" % float(np.abs(got_pool - ref_pool).max()))

    if torch.cuda.is_available():
        gpu = ST.synth_scan_batch(np.array(poses), cfg, discs, radii, [poly],
                                  device="cuda")
        gpu = gpu.cpu().numpy()
        agree_gpu = np.isfinite(gpu) == np.isfinite(got)
        both_gpu = np.isfinite(gpu) & np.isfinite(got)
        d = float(np.abs(gpu[both_gpu] - got[both_gpu]).max()) if both_gpu.any() else 0.0
        # Not bit-identical, and should not be expected to be: CPU and CUDA
        # kernels reduce in different orders and fuse differently in float32.
        # Measured ~1.6e-4 m over 30 m ranges, which is 1e-4 of the sensor's
        # own 0.015 m noise -- far below anything the policy can resolve.
        check("CUDA and CPU agree to float32 reduction-order precision",
              bool(np.all(agree_gpu)) and d < 1e-3, "worst %.3e m" % d)

        # Does the chunked minimum actually hold memory down at the batch size
        # Phase 4 wants? 4096 x 898 as one (E,B,K) tensor would be ~588 MB.
        torch.cuda.reset_peak_memory_stats()
        big = np.zeros((4096, 3))
        big[:, 0] = np.linspace(-15, 15, 4096)
        full = SS.ScanCfg()
        out = ST.synth_scan_batch(big, full, discs, radii, [poly], device="cuda")
        peak = torch.cuda.max_memory_allocated() / 2 ** 20
        check("4096 envs x 898 beams stays under 512 MB of VRAM",
              peak < 512.0, "peak %.0f MB" % peak)
        check("4096-env scan has the right shape", tuple(out.shape) == (4096, 898))
        print("       (4096-env full-resolution scan peaked at %.0f MB)" % peak)
    else:
        print("  [SKIP] no CUDA here; GPU parity and the VRAM budget untested")


# ============================================================= entry point --
SUITES = [
    ("constants", suite_constants),
    ("scan geometry", suite_scan_geometry),
    ("pooling", suite_pooling),
    ("ray primitives", suite_ray_primitives),
    ("oracle", suite_oracle),
    ("fov asymmetry", suite_fov_asymmetry),
    ("kinematics", suite_kinematics),
    ("actions", suite_actions),
    ("observation", suite_observation),
    ("meta", suite_meta),
    ("boundaries", suite_boundaries),
    ("course meta", suite_course_meta),
    ("torch", suite_torch),
]
SLOW = {"oracle"}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--quick", action="store_true",
                    help="skip the brute-force oracle suite")
    args = ap.parse_args(argv)

    print("validate_isaac_obs: numpy %s, python %d.%d"
          % (np.__version__, sys.version_info[0], sys.version_info[1]))
    print("observation: %d pooled beams x %d frames + 7 proprio = %d dims"
          % (OC.N_POOLED, OC.N_HISTORY, OC.OBS_DIM))

    for name, fn in SUITES:
        if args.quick and name in SLOW:
            print("\n[skipped] %s (--quick)" % name)
            continue
        fn()

    failed = [r for r in RESULTS if not r[1]]
    print("\n%d checks, %d failed" % (len(RESULTS), len(failed)))
    for name, _, detail in failed:
        print("  FAIL %s   %s" % (name, detail))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
