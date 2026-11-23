"""The IGVC course as a reinforcement learning environment.

WHY DIRECT RATHER THAN MANAGER-BASED
    The plan called for the manager-based workflow, for its curriculum and
    event managers. In the event every observation and reward term here is
    bespoke and needs the course tensors, so the manager split would have been
    indirection without reuse -- a dozen term functions all reaching back into
    a custom env for state. DirectRLEnv expresses the same environment in far
    less code, and the three-stage curriculum is about fifteen lines either
    way. The deviation is deliberate and recorded here rather than silent.

WHAT THE POLICY SEES AND DOES
    Defined once, in research/isaac/obs_core.py, and shared with the ROS node
    this will deploy as. 128 pooled beams x 3 stacked frames plus velocity,
    goal and previous action: 405 numbers. Two actions in [-1, 1] become
    (linear.x, angular.z), rate limited to the real controller's acceleration
    limits, then wheel velocity targets.

THE SCENE IS ALMOST EMPTY ON PURPOSE
    A ground plane and a robot. No barrels, no lane geometry. The policy meets
    the course only through rays, and lane paint has no collider to meet in
    any case -- it is paint. So the course lives in tensors (course_bank.py)
    and a reset is an index change rather than a scene edit. The ramp is the
    one thing that has to be real, and it is deferred to the curriculum's
    final stage.

FRAMES
    Courses are stored in one world frame and the environments are cloned
    across a grid, so the robot's pose is converted into course-local
    coordinates by subtracting its environment origin. Geometry stays put;
    only the robot moves between frames.
"""
from __future__ import annotations

import math
import os
import sys

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

_ISAAC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                          "..", "..", "..", ".."))
if _ISAAC_DIR not in sys.path:
    sys.path.insert(0, _ISAAC_DIR)

import course_bank as CB          # noqa: E402
import obs_core as OC             # noqa: E402
import scan_sim_torch as ST       # noqa: E402
from scan_sim import ScanCfg, ScanNoise   # noqa: E402

from ...assets.esda_robot import (  # noqa: E402
    ESDA_ROBOT_CFG,
    SPAWN_HEIGHT,
    WHEEL_COMMAND_SIGN,
    apply_friction_materials,
)

# 200 Hz physics with decimation 25 gives an 8 Hz control rate -- exactly
# nav2_params.yaml's controller_frequency, so a step here means what a step
# means on the robot.
PHYSICS_DT = 1.0 / 200.0
DECIMATION = 25


@configclass
class EsdaCourseEnvCfg(DirectRLEnvCfg):
    """Configuration for the course-following task."""

    decimation = DECIMATION
    episode_length_s = 40.0

    action_space = OC.ACTION_DIM
    observation_space = OC.OBS_DIM
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=PHYSICS_DT, render_interval=DECIMATION)

    # 50 m spacing clears the 43 x 37 m arena, so neighbouring environments
    # cannot see or touch each other.
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096, env_spacing=50.0, replicate_physics=True)

    robot: ArticulationCfg = ESDA_ROBOT_CFG.replace(
        prim_path="/World/envs/env_.*/Robot")

    # --- course bank ---
    courses: tuple = CB.DEFAULT_COURSES
    courses_per_layout: int = 4
    course_seed: int = 0
    # The real course is the EVALUATION course. Training on it would measure
    # memorisation of 34 barrel positions.
    include_igvc: bool = False

    # --- task shaping ---
    goal_lookahead_m: float = 4.0
    # Lane range plus margin; beyond this the boundary cannot be seen anyway.
    scan_window_samples: int = 30

    # --- rewards ---
    #
    # Scaled so progress reads as SPEED IN m/s: one control step at the 1.0 m/s
    # cap advances 0.125 m, so 8.0 makes a full-speed step worth about +1.
    #
    # The first attempt used 1.0 and the task was unlearnable: progress was
    # +0.06 a step at half speed while the lane penalty was -0.25, so staying
    # alive cost more than moving earned and episode reward fell as the policy
    # got better at surviving. The terminal penalties are sized against a
    # ~320 step episode worth roughly +300, so a crash costs about a sixth of
    # a perfect run -- enough to matter, not so much that the policy learns to
    # stand still rather than risk one.
    rew_progress: float = 8.0
    rew_lane: float = -0.3
    rew_action_rate: float = -0.05
    rew_spin: float = -0.02
    rew_time: float = -0.02
    rew_collision: float = -50.0
    rew_lane_cross: float = -25.0

    # --- terminations ---
    tilt_limit_rad: float = 0.6

    # --- observation noise, raised by the curriculum ---
    noise_dropout: float = 0.02
    noise_lane_dropout: float = 0.08
    noise_spurious: float = 0.005
    noise_range_bias: float = 0.02


class EsdaCourseEnv(DirectRLEnv):
    cfg: EsdaCourseEnvCfg

    def __init__(self, cfg: EsdaCourseEnvCfg, render_mode: str | None = None, **kw):
        super().__init__(cfg, render_mode, **kw)

        dev = self.device
        bank = CB.build_bank(names=cfg.courses, per_course=cfg.courses_per_layout,
                             seed=cfg.course_seed, include_igvc=cfg.include_igvc)
        t = lambda a: torch.as_tensor(a, dtype=torch.float32, device=dev)  # noqa: E731
        self.bank_centre = t(bank["centre"])           # (C, S, 2)
        self.bank_widths = t(bank["widths"])           # (C, S)
        self.bank_arclen = t(bank["arclen"])           # (C, S)
        self.bank_length = t(bank["length"])           # (C,)
        self.bank_left = t(bank["left"])               # (C, S, 2)
        self.bank_right = t(bank["right"])             # (C, S, 2)
        self.bank_discs = t(bank["discs"])             # (C, K, 2)
        self.bank_radii = t(bank["radii"])             # (C, K)
        self.n_courses = self.bank_centre.shape[0]
        self.n_samples = self.bank_centre.shape[1]
        self.course_names = bank["names"]

        n = self.num_envs
        self.course_idx = torch.zeros(n, dtype=torch.long, device=dev)
        self.prev_action = torch.zeros(n, OC.ACTION_DIM, device=dev)
        self.twist = torch.zeros(n, 2, device=dev)
        self.last_s = torch.zeros(n, device=dev)
        self.frames = torch.zeros(n, OC.N_HISTORY, OC.OBS_PER_FRAME, device=dev)
        self.lap_progress = torch.zeros(n, device=dev)

        self.scan_cfg = ScanCfg()
        self.noise = ScanNoise(range_std=OC.NOISE_STD,
                               dropout_p=cfg.noise_dropout,
                               lane_dropout_p=cfg.noise_lane_dropout,
                               spurious_p=cfg.noise_spurious,
                               range_bias_std=cfg.noise_range_bias)

        names = self.robot.data.joint_names
        self.j_left = names.index("left_wheel_joint")
        self.j_right = names.index("right_wheel_joint")
        self.joint_target = torch.zeros_like(self.robot.data.joint_vel)

        self.env_xy = self.scene.env_origins[:, :2]

    # ------------------------------------------------------------- scene --
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self.robot

        ground = sim_utils.GroundPlaneCfg(
            # The real course ground is mu 100 / mu2 50 in model.sdf. Isaac's
            # default 0.5 lets the wheels slip through every turn.
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=1.0, restitution=0.0))
        ground.func("/World/ground", ground)

        self.scene.clone_environments(copy_from_source=False)
        sim_utils.DomeLightCfg(intensity=1500.0).func(
            "/World/light", sim_utils.DomeLightCfg(intensity=1500.0))

        # PhysX ignores materials baked into the USD, so bind them per clone.
        apply_friction_materials("/World/envs")

    # ------------------------------------------------------------ helpers --
    def _local_pose(self):
        """Robot pose in course coordinates, plus heading."""
        xy = self.robot.data.root_pos_w[:, :2] - self.env_xy
        q = self.robot.data.root_quat_w            # XYZW, not WXYZ
        x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return xy, yaw

    def _nearest(self, xy):
        """Index of the closest centreline sample, and the signed lateral offset.

        Signed so that lane keeping can tell which side it is drifting to, and
        so crossing a boundary is a sign-independent magnitude test.
        """
        centre = self.bank_centre[self.course_idx]            # (n, S, 2)
        d = centre - xy[:, None, :]
        dist = torch.linalg.norm(d, dim=2)
        idx = torch.argmin(dist, dim=1)
        rows = torch.arange(self.num_envs, device=self.device)

        nxt = (idx + 1) % self.n_samples
        tang = centre[rows, nxt] - centre[rows, idx]
        tang = tang / torch.clamp(torch.linalg.norm(tang, dim=1, keepdim=True), min=1e-6)
        left_n = torch.stack([-tang[:, 1], tang[:, 0]], dim=1)
        rel = xy - centre[rows, idx]
        lateral = (rel * left_n).sum(dim=1)
        return idx, lateral, rows

    def _window_segments(self, idx, rows):
        """Boundary segments near the robot, as a fixed-size batch.

        Casting against all 512 boundary segments of every course would dwarf
        the obstacle work for no benefit: the camera cone that produces lane
        returns reaches 10 m, and the centreline is sampled every ~0.5 m. A
        +/-30 sample window covers roughly +/-15 m, comfortably more than can
        be seen, and keeps the raycast batch rectangular.
        """
        w = self.cfg.scan_window_samples
        offs = torch.arange(-w, w + 1, device=self.device)
        gather = (idx[:, None] + offs[None, :]) % self.n_samples   # (n, 2w+1)
        left = self.bank_left[self.course_idx[:, None], gather]    # (n, 2w+1, 2)
        right = self.bank_right[self.course_idx[:, None], gather]
        p0 = torch.cat([left[:, :-1], right[:, :-1]], dim=1)
        p1 = torch.cat([left[:, 1:], right[:, 1:]], dim=1)
        return p0, p1

    def _goal(self, idx, rows):
        """A point further along the centreline. The policy steers to it."""
        step = max(1, int(round(self.cfg.goal_lookahead_m /
                                max(float(self.bank_length.mean()) / self.n_samples, 1e-3))))
        gi = (idx + step) % self.n_samples
        # Index the course dimension with course_idx, NOT rows. Using rows
        # here is only in range while num_envs <= n_courses, so it hides in
        # small smoke tests and asserts on the GPU at training scale.
        return self.bank_centre[self.course_idx, gi]

    # ------------------------------------------------------------ stepping --
    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.clamp(-1.0, 1.0)

        v = OC.V_MIN + 0.5 * (self.actions[:, 0] + 1.0) * (OC.V_MAX - OC.V_MIN)
        w = self.actions[:, 1] * OC.W_MAX
        dt = self.cfg.decimation * self.cfg.sim.dt
        # Rate limit exactly as diff_drive_controller does, so the policy
        # learns against the clamping it will actually meet.
        v = torch.clamp(v, self.twist[:, 0] - OC.A_LIN * dt,
                        self.twist[:, 0] + OC.A_LIN * dt)
        w = torch.clamp(w, self.twist[:, 1] - OC.A_ANG * dt,
                        self.twist[:, 1] + OC.A_ANG * dt)
        self.twist = torch.stack([v, w], dim=1)

        wl = (v - w * OC.WHEEL_SEPARATION / 2.0) / OC.WHEEL_RADIUS
        wr = (v + w * OC.WHEEL_SEPARATION / 2.0) / OC.WHEEL_RADIUS
        self.joint_target[:, self.j_left] = WHEEL_COMMAND_SIGN * wl
        self.joint_target[:, self.j_right] = WHEEL_COMMAND_SIGN * wr

    def _apply_action(self):
        self.robot.set_joint_velocity_target(self.joint_target)

    # -------------------------------------------------------- observations --
    def _get_observations(self) -> dict:
        xy, yaw = self._local_pose()
        idx, lateral, rows = self._nearest(xy)
        self._idx, self._lateral, self._rows = idx, lateral, rows

        p0, p1 = self._window_segments(idx, rows)
        poses = torch.stack([xy[:, 0], xy[:, 1], yaw], dim=1)
        ranges = ST.synth_scan_batch(
            poses, self.scan_cfg,
            discs=self.bank_discs[self.course_idx],
            disc_radii=self.bank_radii[self.course_idx],
            lane_p0=p0, lane_p1=p1,
            noise=self.noise, device=self.device)
        pooled = ST.pool_scan_batch(ranges) / OC.RANGE_CLIP

        goal = self._goal(idx, rows)
        d = goal - xy
        dist = torch.linalg.norm(d, dim=1)
        bearing = torch.atan2(d[:, 1], d[:, 0]) - yaw

        frame = torch.cat([
            pooled.clamp(0.0, 1.0),
            (self.twist[:, 0] / OC.V_MAX).unsqueeze(1),
            (self.twist[:, 1] / OC.W_MAX).unsqueeze(1),
            torch.cos(bearing).unsqueeze(1),
            torch.sin(bearing).unsqueeze(1),
            (dist.clamp(max=OC.GOAL_CLIP) / OC.GOAL_CLIP).unsqueeze(1),
            self.prev_action,
        ], dim=1)

        self.frames = torch.roll(self.frames, shifts=-1, dims=1)
        self.frames[:, -1] = frame

        # Ground truth clearance for the termination test -- NOT the scan.
        # The scan carries spurious returns on purpose (noise_spurious), and
        # terminating on those kills the episode for a sensor glitch rather
        # than a crash: measured, it ended every episode within about six
        # steps and pinned the reward at the collision penalty. What the
        # policy SEES stays noisy; what the environment JUDGES is exact.
        discs = self.bank_discs[self.course_idx]            # (n, K, 2)
        radii = self.bank_radii[self.course_idx]            # (n, K)
        gap = torch.linalg.norm(discs - xy[:, None, :], dim=2) - radii
        # Padding slots have radius 0; push them out of the way rather than
        # letting an obstacle at the origin look like a hit.
        gap = torch.where(radii > 0, gap, torch.full_like(gap, 1e3))
        self._clearance = gap.min(dim=1).values
        return {"policy": self.frames.reshape(self.num_envs, -1)}

    # ------------------------------------------------------------- rewards --
    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        s = self.bank_arclen[self.course_idx, self._idx]
        length = self.bank_length[self.course_idx]
        # Arc length wraps at the lap boundary; take the shorter signed step so
        # completing a lap reads as progress rather than a huge reversal.
        ds = s - self.last_s
        ds = torch.where(ds > length / 2, ds - length, ds)
        ds = torch.where(ds < -length / 2, ds + length, ds)
        self.last_s = s
        self.lap_progress += ds

        half = self.bank_widths[self.course_idx, self._idx] * 0.5
        lane = (self._lateral.abs() / torch.clamp(half, min=0.1)).clamp(max=2.0)
        action_rate = torch.linalg.norm(self.actions - self.prev_action, dim=1)
        spin = self.twist[:, 1].abs()

        reward = (cfg.rew_progress * ds
                  + cfg.rew_lane * lane
                  + cfg.rew_action_rate * action_rate
                  + cfg.rew_spin * spin
                  + cfg.rew_time)
        reward = reward + cfg.rew_collision * self._collided.float()
        reward = reward + cfg.rew_lane_cross * self._crossed.float()
        self.prev_action = self.actions.clone()
        return reward

    # ---------------------------------------------------------- done flags --
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        half = self.bank_widths[self.course_idx, self._idx] * 0.5
        # Collision: an obstacle closer than the footprint radius nav2 plans
        # with. Measured geometrically; see _get_observations for why not the
        # scan.
        self._collided = self._clearance < OC.ROBOT_RADIUS
        # Crossing a lane line is an IGVC fault, so it ends the episode rather
        # than merely costing reward.
        self._crossed = self._lateral.abs() > half

        q = self.robot.data.root_quat_w
        x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        up_z = 1.0 - 2.0 * (x * x + y * y)
        tipped = up_z < math.cos(self.cfg.tilt_limit_rad)

        died = self._collided | self._crossed | tipped
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return died, timeout

    # ------------------------------------------------------------- resets --
    def _reset_idx(self, env_ids):
        if env_ids is None or len(env_ids) == 0:
            return
        super()._reset_idx(env_ids)
        n = len(env_ids)
        dev = self.device

        self.course_idx[env_ids] = torch.randint(0, self.n_courses, (n,), device=dev)
        start = torch.randint(0, self.n_samples, (n,), device=dev)
        centre = self.bank_centre[self.course_idx[env_ids], start]
        nxt = self.bank_centre[self.course_idx[env_ids],
                               (start + 1) % self.n_samples]
        tang = nxt - centre
        yaw = torch.atan2(tang[:, 1], tang[:, 0])

        root = self.robot.data.default_root_state[env_ids].clone()
        root[:, 0:2] = centre + self.env_xy[env_ids]
        root[:, 2] = SPAWN_HEIGHT
        # XYZW. Getting this wrong once spawned the robot upside down.
        root[:, 3] = 0.0
        root[:, 4] = 0.0
        root[:, 5] = torch.sin(yaw * 0.5)
        root[:, 6] = torch.cos(yaw * 0.5)
        root[:, 7:] = 0.0
        self.robot.write_root_state_to_sim(root, env_ids)

        joints = torch.zeros_like(self.robot.data.joint_pos[env_ids])
        self.robot.write_joint_state_to_sim(joints, joints.clone(), None, env_ids)

        self.prev_action[env_ids] = 0.0
        self.twist[env_ids] = 0.0
        self.frames[env_ids] = 0.0
        self.lap_progress[env_ids] = 0.0
        self.last_s[env_ids] = self.bank_arclen[self.course_idx[env_ids], start]
