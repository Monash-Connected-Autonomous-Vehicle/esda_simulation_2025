#!/usr/bin/env python3
"""The one definition of what the RL policy sees and what its output means.

WHY THIS FILE IS SHARED RATHER THAN REIMPLEMENTED
    The policy is trained in Isaac Lab on Windows (Python 3.12, numpy 2.x) and
    deployed as a ROS 2 node in WSL (Python 3.10, numpy pinned <2 so cv_bridge
    keeps working).  Two implementations of "pool 898 beams down to 120" will
    agree right up until one of them is edited, and the symptom -- a policy that
    is flawless in Isaac and wanders in Gazebo -- looks nothing like its cause.

    So the encoder lives here once, in plain numpy, and both sides import it.
    Nothing in this module may import torch, rclpy, isaaclab, cv2 or yaml.
    tools/validate_isaac_obs.py enforces that.

    The torch twin used during training is research/isaac/scan_sim_torch.py.
    It exists only because training needs batched GPU tensors, and the harness
    checks it against this module element by element.

WHAT THE POLICY SEES
    A synthetic /scan_fused, not a bare lidar.  On the real robot the lane
    detection base class projects camera-detected paint into the LiDAR scan and
    republishes it as /scan_fused, and that -- not /scan -- is what
    nav2_params.yaml feeds to every costmap and to AMCL.  Reproducing the bare
    lidar would train a policy that cannot see lane markings at all.

    Lane returns are NOT injected over the full 360 degrees.  They come from a
    camera with a 1.089 rad horizontal FOV and a depth image clipped at 10 m
    (camera.xacro), so beyond that cone there is no paint in /scan_fused no
    matter how much paint is on the ground.  scan_sim.py honours that.

OBSERVATION LAYOUT (one frame, then N_HISTORY frames concatenated oldest first)
        [0 : N_POOLED]   pooled scan, clipped at RANGE_CLIP and scaled to [0,1]
        [N_POOLED + 0]   body linear velocity  / V_MAX
        [N_POOLED + 1]   body angular velocity / W_MAX
        [N_POOLED + 2]   cos of goal bearing in the body frame
        [N_POOLED + 3]   sin of goal bearing in the body frame
        [N_POOLED + 4]   goal distance, clipped at GOAL_CLIP and scaled to [0,1]
        [N_POOLED + 5]   previous action[0], raw in [-1, 1]
        [N_POOLED + 6]   previous action[1], raw in [-1, 1]

ACTION
    Two values in [-1, 1] -- a tanh output -- mapped to (linear.x, angular.z)
    and then rate limited to the acceleration the real diff_drive_controller
    enforces, so the policy learns against the clamping it will actually meet.
    The envelope matches nav2_params.yaml's MPPI limits, not the hardware's:
    MPPI allows vx up to 1.0 and wz up to 0.8, the hardware tolerates 2.0/2.0.
"""
import math

import numpy as np

# --------------------------------------------------------------- the lidar --
# description/lidar.xacro, the gpu_lidar named "laser_frame" that publishes
# /scan (via /scan_raw and scripts/scan_tilt_filter.py).
N_BEAMS_RAW = 898
ANGLE_MIN = -3.14159
ANGLE_MAX = 3.13741
RANGE_MIN = 0.1
RANGE_MAX = 100.0
NOISE_STD = 0.015
SCAN_RATE_HZ = 10.0

# ------------------------------------------------------------- the camera --
# description/camera.xacro: the depth sensor is what projects lane paint into
# /scan_fused, so its cone and clip bound where lane returns can exist at all.
CAMERA_HFOV = 1.089
CAMERA_HALF_FOV = CAMERA_HFOV / 2.0
CAMERA_RANGE_MIN = 0.1
CAMERA_RANGE_MAX = 10.0

# ------------------------------------------------------- the encoded scan --
#
# 898 is an awkward number: 898 = 2 x 449 and 449 is prime, so its only
# divisors are 1, 2, 449 and 898.  Any "round" pooling width would need uneven
# buckets, which are fiddly to vectorise in torch and easy to implement
# differently on the two sides -- the exact drift this module exists to stop.
# Dropping the last TRIM_BEAMS beams leaves 896 = 128 x 7, which reshapes
# exactly in both numpy and torch.  Those beams sit at the very end of the
# sweep, 0.014 rad of the rear-facing arc, where nothing the policy acts on
# lives.
N_POOLED = 128
TRIM_BEAMS = 2
BEAMS_PER_BUCKET = (N_BEAMS_RAW - TRIM_BEAMS) // N_POOLED   # 7
N_HISTORY = 3
# Local costmap in nav2_params.yaml is 10 x 10 m, so beyond 10 m the incumbent
# planner cannot act either. Clipping here keeps the observation's dynamic
# range where the decisions are.
RANGE_CLIP = 10.0
GOAL_CLIP = 10.0

# ------------------------------------------------------------ the vehicle --
# config/my_controllers_2wd.yaml, cross-checked against
# description/robot_core_ref.xacro (wheel_offset_y 0.25 -> separation 0.5).
WHEEL_RADIUS = 0.1625
WHEEL_SEPARATION = 0.5

# config/nav2_params.yaml MPPI limits. Reverse is allowed a little because
# behaviour_tree.py backs out of dead ends and MPPI's vx_min: 0.0 cannot.
V_MAX = 1.0
V_MIN = -0.2
W_MAX = 0.8
# diff_drive_controller acceleration limits, my_controllers_2wd.yaml.
A_LIN = 2.0
A_ANG = 2.0

# nav2_params.yaml controller_frequency.
CONTROL_HZ = 8.0
CONTROL_DT = 1.0 / CONTROL_HZ

# config/nav2_params.yaml, both costmaps.
ROBOT_RADIUS = 0.45

OBS_PER_FRAME = N_POOLED + 7
OBS_DIM = OBS_PER_FRAME * N_HISTORY
ACTION_DIM = 2


# ------------------------------------------------------------------ scans --
def beam_angles(n_beams=N_BEAMS_RAW):
    """Beam bearings in the sensor frame, matching lidar.xacro exactly."""
    return np.linspace(ANGLE_MIN, ANGLE_MAX, int(n_beams), dtype=np.float64)


def sanitise(ranges, range_clip=RANGE_CLIP):
    """Finite, clipped ranges.

    Non-finite entries mean "no return" and become maximum range.  ROS
    LaserScan uses +inf for that and Gazebo emits it freely; left alone numpy
    propagates it through everything downstream, and a single NaN turns the
    whole observation -- and then the policy output -- into NaN.
    """
    r = np.asarray(ranges, dtype=np.float64).ravel()
    r = np.where(np.isfinite(r), r, range_clip)
    return np.clip(r, 0.0, range_clip)


def pool_scan(ranges, range_clip=RANGE_CLIP):
    """898 raw beams -> N_POOLED, by MINIMUM within each bucket.

    Minimum, not mean.  A mean lets a 0.3 m barrel vanish between two free
    beams, and the entire point of the scan is not to hit things.  The cost is
    a conservative policy, which is the right side to err on.

    Requires exactly N_BEAMS_RAW beams.  That is deliberate: a real
    /scan_fused with a different beam count is a genuine train/deploy
    mismatch, not something to paper over silently.  Use resample_scan when
    you have decided that resampling is what you want.
    """
    r = sanitise(ranges, range_clip)
    if r.shape[0] != N_BEAMS_RAW:
        raise ValueError(
            "expected %d beams (description/lidar.xacro), got %d -- use "
            "resample_scan if you really mean to resample"
            % (N_BEAMS_RAW, r.shape[0]))
    return r[:N_BEAMS_RAW - TRIM_BEAMS].reshape(N_POOLED, BEAMS_PER_BUCKET).min(axis=1)


def resample_scan(ranges, n_out=N_POOLED, range_clip=RANGE_CLIP):
    """Reduce ANY beam count to n_out by minimum over contiguous buckets.

    The fallback for a scan that is not 898 beams.  Bucket edges are floored
    from exact fractions, so sizes differ by at most one beam and the mapping
    is fully determined by (n_raw, n_out) -- two callers with the same inputs
    always agree.  For n_raw == N_BEAMS_RAW this does NOT reproduce pool_scan,
    because pool_scan trims the two rear beams first; that is why pool_scan is
    the one wired into the observation and this is the escape hatch.
    """
    r = sanitise(ranges, range_clip)
    n_raw = r.shape[0]
    if n_raw == 0:
        raise ValueError("empty scan")
    edges = (np.arange(n_out + 1, dtype=np.float64) * n_raw / n_out).astype(int)
    edges[-1] = n_raw
    out = np.empty(n_out, dtype=np.float64)
    for i in range(n_out):
        lo = min(edges[i], n_raw - 1)
        hi = max(lo + 1, edges[i + 1])
        out[i] = r[lo:hi].min()
    return out


def normalise_scan(pooled, range_clip=RANGE_CLIP):
    return np.clip(np.asarray(pooled, dtype=np.float64) / range_clip, 0.0, 1.0)


# ------------------------------------------------------------------- goal --
def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return (np.asarray(a, dtype=np.float64) + np.pi) % (2.0 * np.pi) - np.pi


def goal_features(robot_xy, robot_yaw, goal_xy, goal_clip=GOAL_CLIP):
    """(cos bearing, sin bearing, normalised distance) in the body frame.

    cos/sin rather than the angle itself: the angle is discontinuous at +/-pi,
    which is exactly where a robot that needs to turn around sits.
    """
    dx = float(goal_xy[0]) - float(robot_xy[0])
    dy = float(goal_xy[1]) - float(robot_xy[1])
    dist = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx) - float(robot_yaw)
    return np.array([math.cos(bearing), math.sin(bearing),
                     min(dist, goal_clip) / goal_clip], dtype=np.float64)


# ---------------------------------------------------------------- actions --
def action_to_twist(action, prev_twist=None, dt=CONTROL_DT):
    """[-1,1]^2 -> (linear.x, angular.z), rate limited like the real controller.

    Without the rate limit the policy would learn to step from full reverse to
    full forward in one control tick, which diff_drive_controller's
    linear.x.max_acceleration 2.0 would then refuse to execute -- a silent
    train/deploy mismatch that shows up only as degraded real-world behaviour.
    """
    a = np.asarray(action, dtype=np.float64).ravel()
    if a.shape[0] != ACTION_DIM:
        raise ValueError("action must have %d elements, got %d" % (ACTION_DIM, a.shape[0]))
    a = np.clip(a, -1.0, 1.0)
    v = V_MIN + 0.5 * (a[0] + 1.0) * (V_MAX - V_MIN)
    w = a[1] * W_MAX
    if prev_twist is not None:
        pv, pw = float(prev_twist[0]), float(prev_twist[1])
        v = min(max(v, pv - A_LIN * dt), pv + A_LIN * dt)
        w = min(max(w, pw - A_ANG * dt), pw + A_ANG * dt)
    return np.array([v, w], dtype=np.float64)


def twist_to_action(twist):
    """Inverse of action_to_twist with prev_twist=None. Used by tests."""
    v, w = float(twist[0]), float(twist[1])
    a0 = 2.0 * (v - V_MIN) / (V_MAX - V_MIN) - 1.0
    a1 = w / W_MAX
    return np.clip(np.array([a0, a1], dtype=np.float64), -1.0, 1.0)


def twist_to_wheels(v, w, radius=WHEEL_RADIUS, separation=WHEEL_SEPARATION):
    """(linear.x, angular.z) -> (left, right) wheel angular velocity, rad/s."""
    return np.array([(v - w * separation / 2.0) / radius,
                     (v + w * separation / 2.0) / radius], dtype=np.float64)


def wheels_to_twist(wl, wr, radius=WHEEL_RADIUS, separation=WHEEL_SEPARATION):
    """(left, right) wheel angular velocity -> (linear.x, angular.z)."""
    return np.array([radius * (wl + wr) / 2.0,
                     radius * (wr - wl) / separation], dtype=np.float64)


# ----------------------------------------------------------- the full obs --
def encode_frame(pooled_scan, twist, robot_xy, robot_yaw, goal_xy, prev_action):
    """One observation frame. See the layout in the module docstring."""
    out = np.empty(OBS_PER_FRAME, dtype=np.float64)
    out[:N_POOLED] = normalise_scan(pooled_scan)
    out[N_POOLED + 0] = float(twist[0]) / V_MAX
    out[N_POOLED + 1] = float(twist[1]) / W_MAX
    out[N_POOLED + 2:N_POOLED + 5] = goal_features(robot_xy, robot_yaw, goal_xy)
    out[N_POOLED + 5:N_POOLED + 7] = np.clip(
        np.asarray(prev_action, dtype=np.float64).ravel(), -1.0, 1.0)
    return out


def stack_history(frames):
    """N_HISTORY frames, oldest first, into the flat observation vector."""
    frames = list(frames)
    if len(frames) != N_HISTORY:
        raise ValueError("expected %d frames, got %d" % (N_HISTORY, len(frames)))
    return np.concatenate(frames).astype(np.float64)


# -------------------------------------------------- the deployment contract --
def policy_meta():
    """Everything a deployment must agree with the trained policy about.

    export_policy.py writes this beside policy.onnx; rl_policy_node.py loads it
    and calls check_meta before running anything.  A policy trained with
    N_POOLED 120 and loaded by a node that pools to 96 would otherwise produce
    confident, wrong velocities rather than an error.
    """
    return {
        "obs_dim": int(OBS_DIM),
        "obs_per_frame": int(OBS_PER_FRAME),
        "action_dim": int(ACTION_DIM),
        "n_pooled": int(N_POOLED),
        "n_history": int(N_HISTORY),
        "n_beams_raw": int(N_BEAMS_RAW),
        "trim_beams": int(TRIM_BEAMS),
        "beams_per_bucket": int(BEAMS_PER_BUCKET),
        "angle_min": float(ANGLE_MIN),
        "angle_max": float(ANGLE_MAX),
        "range_clip": float(RANGE_CLIP),
        "goal_clip": float(GOAL_CLIP),
        "pooling": "min",
        "v_min": float(V_MIN),
        "v_max": float(V_MAX),
        "w_max": float(W_MAX),
        "a_lin": float(A_LIN),
        "a_ang": float(A_ANG),
        "control_hz": float(CONTROL_HZ),
        "wheel_radius": float(WHEEL_RADIUS),
        "wheel_separation": float(WHEEL_SEPARATION),
        "cmd_vel_topic": "/cmd_vel",
        "scan_topic": "/scan_fused",
        "odom_topic": "/odometry/filtered",
    }


def check_meta(loaded):
    """Return a list of disagreements between `loaded` and this module.

    Empty list means the deployment matches what was trained.  Returning the
    differences rather than raising lets the caller report all of them at once.
    """
    mine = policy_meta()
    problems = []
    for key, want in sorted(mine.items()):
        if key not in loaded:
            problems.append("missing key %r (expected %r)" % (key, want))
            continue
        got = loaded[key]
        if isinstance(want, float):
            if not math.isclose(float(got), want, rel_tol=1e-9, abs_tol=1e-12):
                problems.append("%s: policy %r, this code %r" % (key, got, want))
        elif got != want:
            problems.append("%s: policy %r, this code %r" % (key, got, want))
    return problems
