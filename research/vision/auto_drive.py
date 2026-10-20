#!/usr/bin/env python3
"""
auto_drive.py — drive the course with NO waypoints and no goal from a human.

This is the IGVC Auto-Nav behaviour: the robot works out where to go from the
lane boundaries it can see, and keeps going. Nothing tells it a destination.

HOW IT WORKS

    lane_seg_node.py  ->  /lane/points  (continuous lane boundaries, base_link)
                                 |
                          auto_drive.py   picks a "carrot" on the lane
                                 |        centreline a few metres ahead
                                 v
                            /goal_pose  ->  Nav2
                                 |
                            /cmd_vel -> twist_mux -> wheels

    Nav2's NavigateToPoseNavigator subscribes to /goal_pose, so publishing a
    PoseStamped there starts a navigation. Republish a carrot that keeps
    sliding forward and the robot drives continuously, while Nav2 still does
    the obstacle avoidance, footprint checking and recovery behaviours.

WHY A CARROT RATHER THAN A DIRECT STEERING CONTROLLER

    A pure-pursuit controller writing straight to /cmd_vel would be fewer
    moving parts, but it would throw away the costmap: the barrels seen by
    the lidar and by the cone layer would stop being avoided, and the robot
    would steer into anything that is not a painted line. Keeping Nav2 in the
    loop means the lane tells it WHERE to aim and the costmap still decides
    what is safe on the way.

WHY THE CENTRELINE IS BUILT FROM BANDS

    /lane/points is a cloud, not two tidy polylines, and either boundary can
    be missing — on a left-hand bend the right line leaves the camera frame
    entirely. So for each distance band ahead the nearest point on each side
    is taken; with both, the centre is their midpoint; with one, the centre
    is half a lane width inboard of it. Losing a boundary degrades the aim
    rather than breaking it.

USAGE
    python3 auto_drive.py                      # defaults
    python3 auto_drive.py --lookahead 3.5 --lane-width 3.0
    python3 auto_drive.py --dry-run            # print carrots, publish nothing
"""
import argparse
import math

import numpy as np

# ROS is needed to RUN this; the steering geometry below is plain numpy so
# test_auto_drive.py can exercise it with no simulator and no ROS installed.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, LaserScan
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from tf2_ros import Buffer, TransformListener
    HAVE_ROS = True
except ImportError:                                   # offline test run
    HAVE_ROS = False
    Node = object


def cloud_xyz(msg):
    """PointCloud2 (xyz float32, point_step 12) -> Nx3 float array.

    Parsed directly rather than through sensor_msgs_py, which is a separate
    package that may not be installed, and after the cv_bridge/NumPy-2 ABI
    break this pipeline has already paid for one avoidable dependency.
    """
    if msg.point_step < 12 or len(msg.data) < msg.point_step:
        return np.empty((0, 3), np.float32)
    n = len(msg.data) // msg.point_step
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step)
    raw = raw.reshape(n, msg.point_step)[:, :12]
    return np.ascontiguousarray(raw).view(np.float32).reshape(n, 3)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_points(msg, tf, target):
    """LaserScan -> Nx3 points in `target`, dropping invalid returns.

    The lidar is the one sensor that cannot be fooled by appearance. YOLO can
    miss a barrel -- it is a learned model on renders it was not trained on --
    but a solid object in front of a lidar always comes back. Using both means
    a missed detection degrades the aim instead of steering into something.
    """
    try:
        t = tf.lookup_transform(target, msg.header.frame_id, rclpy.time.Time())
    except Exception:                                   # noqa: BLE001
        return np.empty((0, 3), np.float32)
    r = np.asarray(msg.ranges, dtype=np.float32)
    if r.size == 0:
        return np.empty((0, 3), np.float32)
    ang = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
    keep = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    r, ang = r[keep], ang[keep]
    if r.size == 0:
        return np.empty((0, 3), np.float32)
    x, y = r * np.cos(ang), r * np.sin(ang)
    th = yaw_of(t.transform.rotation)
    c, sn = math.cos(th), math.sin(th)
    X = t.transform.translation.x + c * x - sn * y
    Y = t.transform.translation.y + sn * x + c * y
    return np.stack([X, Y, np.zeros_like(X)], axis=1).astype(np.float32)


from lane_side import one_sided_lane, view_half_at   # pure, no ROS


def band_gap(lane_ys, cone_ys, lane_w, side_gate, half_w, cone_r,
             blind_half=0.0, view_half=0.0):
    """Centre of the widest SAFE gap across one distance band, or None.

    The lane gives an interval the robot may occupy; every cone punches a
    hole in it. Picking the middle of the lane ignores the holes, which is
    how you aim a goal straight at a barrel -- the planner then either takes
    an enormous detour or runs its recovery behaviours and reverses. Picking
    the middle of the widest REMAINING interval threads the gap instead, and
    falls out naturally into slalom behaviour when barrels alternate sides.
    """
    left = lane_ys[lane_ys > side_gate]
    right = lane_ys[lane_ys < -side_gate]
    if left.size and right.size:
        lo, hi = float(right.max()), float(left.min())
    elif left.size:
        # One line only. Which EDGE of the lane is it? Getting this wrong is
        # how the robot ends up parallel to the course and outside it --
        # see lane_side.py. view_half 0.0 keeps the original assumption.
        (lo, hi), _ = one_sided_lane(float(left.min()), True, lane_w, view_half)
    elif right.size:
        (lo, hi), _ = one_sided_lane(float(right.max()), False, lane_w, view_half)
    elif blind_half > 0.0:
        # No paint in this band. Rather than coast blind or stall, treat a
        # corridor of +-blind_half ahead as drivable and let the obstacles --
        # here mostly lidar returns -- carve the gap out of it. Measured in
        # the drive log: the robot sat for ~10 s repeating "refusing to coast:
        # 6..14 lidar return(s) in the path" while holding exactly the data
        # needed to steer around them.
        lo, hi = -blind_half, blind_half
    else:
        return None

    # inset by the robot's half width so an interval that "fits" really does
    free = [(lo + half_w, hi - half_w)]
    if free[0][1] <= free[0][0]:
        return None

    block = cone_r + half_w
    for yc in cone_ys:
        nxt = []
        b0, b1 = yc - block, yc + block
        for a, b in free:
            if b1 <= a or b0 >= b:
                nxt.append((a, b))
                continue
            if b0 > a:
                nxt.append((a, min(b, b0)))
            if b1 < b:
                nxt.append((max(a, b1), b))
        free = [(a, b) for a, b in nxt if b - a > 1e-6]
        if not free:
            return None

    a, b = max(free, key=lambda seg: seg[1] - seg[0])
    return float(0.5 * (a + b))


def lane_centre(pts, x_lo, x_hi, step, lane_w, side_gate,
                cones=None, half_w=0.35, cone_r=0.35, blind_half=0.0,
                max_step=0.6, fov_deg=0.0):
    """Drivable line ahead of the robot, in base_link: [(x, y), ...] near first.

    Stops at the first band with no safe gap rather than looking past it --
    aiming beyond a wall is what produced the detours.
    """
    out = []
    if len(pts) == 0 and blind_half <= 0.0:
        return out
    cones = np.empty((0, 3), np.float32) if cones is None else cones
    x, y = pts[:, 0], pts[:, 1]
    cx = cones[:, 0] if len(cones) else np.empty(0)
    cy = cones[:, 1] if len(cones) else np.empty(0)
    # Top edge excluded on purpose: a band starting AT x_hi would extend to
    # x_hi+step and put the carrot further out than --lookahead asked for.
    for x0 in np.arange(x_lo, x_hi, step):
        m = (x >= x0) & (x < x0 + step)
        if not m.any() and blind_half <= 0.0:
            continue
        cm = (cx >= x0) & (cx < x0 + step) if len(cones) else np.zeros(0, bool)
        c = band_gap(y[m], cy[cm] if len(cones) else np.empty(0),
                     lane_w, side_gate, half_w, cone_r, blind_half,
                     view_half_at(x0 + step / 2.0, fov_deg))
        if c is None:
            break                    # blocked: do not aim past it
        c = float(c)
        # Stop extending across a LATERAL discontinuity. The carrot is the
        # furthest band, so without this a sequence like 0.00, -1.24, 0.00
        # (clear, dodge, clear) would aim at the far 0.00 -- straight through
        # the obstacle the middle band had to swerve around. Breaking here
        # keeps the carrot on the near side of the swerve; as the robot closes
        # in, the dodge becomes the first band and the carrot follows it.
        if out and abs(c - out[-1][1]) > max_step:
            break
        out.append((float(x0 + step / 2.0), c))
    return out


def lidar_corridor(walls, x_lo, x_hi, lane_w, side_gate, half_w, cone_r,
                   max_half=3.0):
    """Drivable line from the LIDAR alone, when there is no paint to follow.

    The failure this exists for: the lane model was trained on synthetic
    frames and goes blind on hard cast shadows. The old behaviour was to
    creep dead ahead for a few seconds and then stop -- safe, in that it does
    not drive off the course, but a robot that stops does not finish.

    FIRST ATTEMPT, AND WHY IT NEVER ONCE FIRED

    It bounded each 0.5 m distance band separately and demanded two returns
    inside that band to form a gap. That is right for a continuous wall and
    wrong for this course: a 360-sample scan put only 28-47 returns in view
    at all, and a barrel field is mostly empty space, so almost every band
    held nought or one return and the function bailed out immediately. It
    was armed for an entire benchmark run and contributed nothing.

    So: pool the evidence over the whole lookahead window instead of slicing
    it, and treat the nearest return on each side as the corridor edge --
    exactly what the paint logic already does with lane boundaries. That is
    band_gap, so this reuses it rather than reimplementing the gap search:
    returns dead ahead are carved out as obstacles, the widest remaining
    interval wins, and a corridor too narrow for the robot is refused.
    """
    if len(walls) == 0:
        return []
    m = ((walls[:, 0] >= x_lo) & (walls[:, 0] < x_hi)
         & (np.abs(walls[:, 1]) <= max_half))
    if not m.any():
        return []
    ys = walls[m, 1]
    ahead = ys[np.abs(ys) <= side_gate]      # things in the robot's path
    c = band_gap(ys, ahead, lane_w, side_gate, half_w, cone_r)
    if c is None:
        return []
    return [(float(0.5 * (x_lo + x_hi)), float(c))]




def should_log_idle(reason_key, last_key, since_last_log, repeat_every=10.0):
    """Log this 'not steering' reason, or stay quiet?

    Dedupe by reason was added because one stop produced 468 identical lines.
    It then did something much worse: the driver stopped dead 175 s into a
    run, and because that same reason had been logged once at STARTUP, the
    recurrence was suppressed and the log simply went silent. Nav2's log said
    "Goal succeeded" and then nothing, and it took reading four logs side by
    side to work out the robot had quietly parked.

    So: always log a CHANGE of state, and while a state persists repeat it
    every repeat_every seconds. Silence must never be ambiguous between
    "fine" and "stopped".
    """
    if reason_key != last_key:
        return True
    return since_last_log >= repeat_every


def blind_limit(since_s, dist_m, ever_sensed, blind_timeout,
                startup_grace, max_blind_dist):
    """Why the driver should stop, or "" to carry on. Pure, so it is testable.

    TWO SEPARATE LIMITS, BOTH LEARNED THE HARD WAY.

    Time. The first version used a 1e9 sentinel for "never sensed", so on the
    very first tick 1e9 > 4 was true and the driver refused to start. Fixing
    that by making "never" mean "do not stop" was worse: on a run where the
    lane node produced nothing at all, the clock never started, and the robot
    drove 214 consecutive blind-corridor carrots in a dead straight line --
    216.8 m, out of a 43 x 37 m arena and 195 m into empty space, reported as
    0% stalled. So "never sensed" gets a generous grace from STARTUP and then
    stops like anything else.

    Distance. Even with the clock running, a robot that briefly glimpses
    paint every few seconds can keep resetting the timer while travelling a
    long way on no real information. Distance since the last genuine reading
    is the honest measure of how far it has gone on faith.
    """
    limit = blind_timeout if ever_sensed else startup_grace
    if since_s is not None and since_s > limit:
        return ("no sensing for %.0fs" % since_s) if ever_sensed else (
            "nothing ever sensed in %.0fs since startup" % since_s)
    if max_blind_dist > 0 and dist_m > max_blind_dist:
        return "%.0f m travelled with no lane reading" % dist_m
    return ""


class LaneMemory:
    """The recent carrots that came from real paint, held in the global frame.

    A dropout of a second or two is the common case -- a shadow, a gap in the
    painting, a frame the model got wrong. Coasting dead ahead through one is
    what takes the robot off a bend: straight is only the right answer on a
    straight. Remembering where the paint said to go, in a frame that does
    not move with the robot, lets it finish the arc it was on instead of
    being yanked straight by the blind corridor.

    A TRAIL, not a single point. One point is passed within a couple of
    seconds and then the memory is empty again exactly when it is needed; a
    trail keeps the whole recent curve, and recall returns the furthest point
    still ahead.

    Deliberately NOT an extrapolation. It never invents road beyond the last
    place the paint was actually seen, and every point expires.
    """

    def __init__(self, max_age=3.0, min_ahead=0.8, max_lateral=2.5,
                 min_sep=0.3, cap=60):
        self.max_age = max_age
        self.min_ahead = min_ahead
        self.max_lateral = max_lateral
        self.min_sep = min_sep
        self.cap = cap
        self.pts = []               # [(t, gx, gy)], oldest first
        self.why_not = "nothing remembered yet"

    def remember(self, t, gx, gy):
        if self.pts:
            _, lx, ly = self.pts[-1]
            if math.hypot(gx - lx, gy - ly) < self.min_sep:
                self.pts[-1] = (t, gx, gy)      # same place, fresher
                return
        self.pts.append((t, gx, gy))
        if len(self.pts) > self.cap:
            self.pts.pop(0)

    def recall(self, t, rx, ry, ryaw):
        """-> (x, y) in base_link: the furthest remembered point still ahead.

        Sets why_not when it returns None, because 'the fallback did not
        fire' is useless in a log and 'all 7 remembered points are behind
        the robot' tells you what to change.
        """
        if not self.pts:
            self.why_not = "nothing remembered yet"
            return None
        c, sn = math.cos(ryaw), math.sin(ryaw)
        n_old = n_behind = n_wide = 0
        best = None
        for (pt, gx, gy) in self.pts:
            if t - pt > self.max_age:
                n_old += 1
                continue
            dx, dy = gx - rx, gy - ry
            x = c * dx + sn * dy
            y = -sn * dx + c * dy
            if x < self.min_ahead:
                n_behind += 1
                continue
            if abs(y) > self.max_lateral:
                n_wide += 1
                continue
            if best is None or x > best[0]:
                best = (float(x), float(y))
        if best is None:
            self.why_not = (f"{len(self.pts)} remembered: {n_old} expired, "
                            f"{n_behind} already passed, {n_wide} off to the side")
        return best

    @property
    def t(self):
        return self.pts[-1][0] if self.pts else None

    def forget(self):
        self.pts = []


class AutoDrive(Node):
    def __init__(self, a):
        super().__init__("auto_drive")
        self.a = a
        self.pts = np.empty((0, 3), np.float32)
        self.cones = np.empty((0, 3), np.float32)
        self.walls = np.empty((0, 3), np.float32)   # lidar returns, base_link
        self._last_lane_t = None
        self._last_sense_t = None
        self.last_cloud_t = None
        self.last_goal = None
        self.last_pose = None
        self.last_move_t = None
        self._t0 = None
        self._blind_dist = 0.0
        self._odo_prev = None
        self._steering = True
        self.mem = LaneMemory(max_age=a.lane_memory,
                              min_ahead=max(0.5, a.lookahead_min * 0.5))

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(PointCloud2, a.lane_topic,
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, a.cone_topic,
                                 self._on_cones, qos_profile_sensor_data)
        self.create_subscription(LaserScan, a.scan_topic,
                                 self._on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseStamped, a.goal_topic, 1)
        self.create_timer(1.0 / a.rate, self._tick)

        self.get_logger().info(
            f"auto_drive: {a.lane_topic} -> {a.goal_topic} in '{a.global_frame}', "
            f"lookahead {a.lookahead_min:.1f}-{a.lookahead:.1f} m, "
            f"lane width {a.lane_width:.1f} m"
            + (f", lane memory {a.lane_memory:.1f}s" if a.lane_memory > 0 else "")
            + (", lidar corridor ON" if a.lidar_corridor else "")
            + ("  [DRY RUN]" if a.dry_run else ""))

    def _on_cloud(self, msg):
        self.pts = cloud_xyz(msg)
        self.last_cloud_t = self.get_clock().now()
        self.cloud_frame = msg.header.frame_id or self.a.robot_frame

    def _on_cones(self, msg):
        self.cones = cloud_xyz(msg)

    def _on_scan(self, msg):
        self.walls = scan_points(msg, self.tf, self.a.robot_frame)

    def _tick(self):
        now = self.get_clock().now()
        stale = (self.last_cloud_t is None or
                 (now - self.last_cloud_t).nanoseconds * 1e-9 > self.a.cloud_timeout)

        # Obstacles are the UNION of what vision calls a cone and what the
        # lidar actually returns. Vision alone was the earlier version's
        # blind spot: a barrel the model misses is invisible to the aim.
        obst = self.cones
        if self.a.use_scan and len(self.walls):
            obst = (self.walls if not len(obst)
                    else np.vstack([obst, self.walls]))

        # Robot pose first. The lane memory lives in the global frame, so it
        # cannot be consulted without knowing where the robot is right now.
        try:
            tr = self.tf.lookup_transform(self.a.global_frame,
                                          self.a.robot_frame,
                                          rclpy.time.Time())
        except Exception as e:                      # noqa: BLE001
            self._idle(f"no TF {self.a.global_frame}<-{self.a.robot_frame}: {e}")
            return
        th = yaw_of(tr.transform.rotation)
        rx = float(tr.transform.translation.x)
        ry = float(tr.transform.translation.y)
        tnow = now.nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now
        # Distance covered since the last genuine sensing, for blind_limit.
        if self._odo_prev is not None:
            self._blind_dist += math.hypot(rx - self._odo_prev[0],
                                           ry - self._odo_prev[1])
        self._odo_prev = (rx, ry)

        def to_global(x, y):
            return (float(rx + math.cos(th) * x - math.sin(th) * y),
                    float(ry + math.sin(th) * x + math.cos(th) * y))

        # Is there any PAINT in the lookahead window, or is the centreline
        # about to come from the blind corridor alone? The distinction got
        # lost in an earlier version: a blind-corridor result logged as
        # "3 band(s)" exactly like a real lane reading, and because an empty
        # corridor always yields dead-ahead, the robot drove in a straight
        # line out of the course and kept going.
        inwin = (self.pts[(self.pts[:, 0] >= self.a.lookahead_min) &
                          (self.pts[:, 0] < self.a.lookahead)]
                 if len(self.pts) else np.empty((0, 3), np.float32))
        blind = len(inwin) == 0
        if not blind:
            self._last_lane_t = now

        centres = [] if stale else lane_centre(
            self.pts, self.a.lookahead_min, self.a.lookahead,
            self.a.band, self.a.lane_width, self.a.side_gate,
            obst, self.a.half_width, self.a.cone_radius,
            self.a.blind_corridor, self.a.max_lateral_step,
            self.a.fov_deg)
        self._n_obst = len(obst)

        # Where to aim, in order of how much the source actually knows.
        #
        #   1. paint            the ground truth the course is marked with
        #   2. lane memory      where the paint last said to go, still valid
        #   3. lidar corridor   no paint, but the barrels bound a gap
        #   4. blind corridor   no information at all -- time-limited
        #
        # 2 and 3 exist because the detector goes blind on cast shadows and
        # the old behaviour was to stop. Stopping is safe and loses the race.
        x = y = None
        why = ""
        if centres and not blind:
            x, y = centres[-1]
            why = (f"{len(centres)} band(s), {len(self.cones)} cone pts, "
                   f"{len(self.walls)} lidar pts")
            self._last_sense_t = now
            self._blind_dist = 0.0
            gx, gy = to_global(x, y)
            self.mem.remember(tnow, gx, gy)
        else:
            recalled = (self.mem.recall(tnow, rx, ry, th)
                        if self.a.lane_memory > 0 else None)
            corr = (lidar_corridor(self.walls, self.a.lookahead_min,
                                   self.a.lookahead, self.a.lane_width,
                                   self.a.side_gate, self.a.half_width,
                                   self.a.cone_radius, self.a.corridor_max)
                    if (self.a.lidar_corridor and self.a.use_scan
                        and len(self.walls)) else [])
            if recalled is not None:
                x, y = recalled
                why = ("lane memory, paint lost %.1fs ago"
                       % (tnow - self.mem.t))
            elif corr:
                x, y = corr[-1]
                why = (f"lidar corridor, {len(corr)} band(s), no paint")
                self._last_sense_t = now
                self._blind_dist = 0.0

        if x is None:
            # Nothing that counts as sensing. Allow the old behaviour --
            # creep straight for a moment, in case it is a gap in the paint
            # -- then stop rather than drive off the course.
            ref = self._last_sense_t or self._t0
            since = (now - ref).nanoseconds * 1e-9
            if self.a.lane_memory > 0 or self.a.lidar_corridor:
                self._note_fallbacks()
            stop = blind_limit(since, self._blind_dist,
                               self._last_sense_t is not None,
                               self.a.blind_timeout, self.a.startup_grace,
                               self.a.max_blind_dist)
            if stop:
                self._idle(f"{stop} -- stopping rather than driving on blind")
                return
            if centres:
                x, y = centres[-1]
                why = f"blind corridor, {len(centres)} band(s)"
            elif stale or not self.a.coast:
                self._idle("no lane points" if stale else "no centreline")
                return
            else:
                x, y = self.a.lookahead_min, 0.0
                # Coasting is for a gap in the paint, not for driving into
                # something. The lidar still works when the paint does not.
                if self.a.use_scan and len(self.walls):
                    w = self.walls
                    ahead = w[(w[:, 0] > 0.0) & (w[:, 0] < x + 1.0) &
                              (np.abs(w[:, 1]) < self.a.half_width + 0.2)]
                    if len(ahead):
                        self._idle(f"refusing to coast: {len(ahead)} lidar "
                                   f"return(s) in the path")
                        return
                why = "coasting straight (no boundary visible, path clear on lidar)"

        # float() at the boundary: anything that passed through numpy is a
        # float32, and geometry_msgs asserts on exact Python float.
        gx, gy = to_global(x, y)
        gth = float(th + math.atan2(y, x))

        # Re-publishing /goal_pose restarts the behaviour tree, so only do it
        # when the aim has actually moved. Without this the BT is torn down
        # and rebuilt several times a second and the robot stutters.
        # ...but a goal that never moves must not be able to wedge the robot.
        # Observed: Nav2 logged "Reached the goal! / Goal succeeded" and then
        # went quiet for the rest of the run. Once the robot stops, the
        # carrot stops moving with it, the shift test suppresses the next
        # publish, and with no new goal the robot has no reason to move --
        # the stillness keeps itself going. Re-sending the same goal costs a
        # behaviour-tree restart, which is only expensive while driving, and
        # while stopped is exactly what is needed.
        moved = (self.last_pose is None or
                 math.hypot(rx - self.last_pose[0], ry - self.last_pose[1])
                 > self.a.stuck_dist)
        if moved:
            self.last_pose, self.last_move_t = (rx, ry), tnow
        stuck_for = tnow - (self.last_move_t or tnow)
        if self.last_goal is not None and stuck_for < self.a.stuck_secs:
            dx, dy = gx - self.last_goal[0], gy - self.last_goal[1]
            if math.hypot(dx, dy) < self.a.min_goal_shift:
                return
        if stuck_for >= self.a.stuck_secs:
            self.get_logger().warn(
                f"not moved {self.a.stuck_dist} m in {stuck_for:.0f}s -- "
                f"re-sending the goal ({why})")
            self.last_move_t = tnow

        msg = PoseStamped()
        msg.header.frame_id = self.a.global_frame
        msg.header.stamp = now.to_msg()
        msg.pose.position.x, msg.pose.position.y = gx, gy
        msg.pose.orientation.z = math.sin(gth / 2.0)
        msg.pose.orientation.w = math.cos(gth / 2.0)
        if not self.a.dry_run:
            self.pub.publish(msg)
        if not getattr(self, "_steering", True):
            self.get_logger().info("steering again")
            self._last_why = None
        self._steering = True
        self.last_goal = (gx, gy)
        self.get_logger().info(
            f"carrot ({x:.2f},{y:+.2f}) base_link -> ({gx:.2f},{gy:.2f}) "
            f"{self.a.global_frame}   [{why}]")

    def _note_fallbacks(self):
        # Log, at most once per distinct reason, WHY the fallbacks declined.
        # Both were armed for a whole 420 s run and fired zero times, and the
        # log said nothing at all about it -- which cost an hour of guessing.
        bits = []
        if self.a.lane_memory > 0:
            bits.append("memory: " + self.mem.why_not)
        if self.a.lidar_corridor:
            bits.append(f"corridor: no gap from {len(self.walls)} lidar pts")
        msg = "fallbacks declined -- " + "; ".join(bits)
        key = "".join(ch for ch in msg if not ch.isdigit())
        if getattr(self, "_last_fb", None) != key:
            self.get_logger().info(msg)
            self._last_fb = key

    def _idle(self, why):
        # Key on the reason WITHOUT its numbers, so "no paint for 44s" and
        # "...45s" are the same state rather than two. Then re-log on change
        # and periodically while it persists -- see should_log_idle.
        key = "".join(c for c in why if not c.isdigit())
        now = self.get_clock().now().nanoseconds * 1e-9
        since = now - getattr(self, "_last_why_t", 0.0)
        if should_log_idle(key, getattr(self, "_last_why", None), since,
                           self.a.idle_log):
            held = ("" if key != getattr(self, "_last_why", None)
                    else f"  (still, {now - getattr(self, '_idle_since', now):.0f}s)")
            self.get_logger().warn(f"not steering: {why}{held}")
            self._last_why_t = now
        if key != getattr(self, "_last_why", None):
            self._idle_since = now
        self._last_why = key
        self._steering = False


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--lane-topic", default="/lane/points")
    p.add_argument("--cone-topic", default="/cone/points")
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--use-scan", action="store_true", default=True,
                   help="treat lidar returns as obstacles too (default on)")
    p.add_argument("--no-scan", dest="use_scan", action="store_false",
                   help="steer from vision alone")
    p.add_argument("--goal-topic", default="/goal_pose")
    p.add_argument("--global-frame", default="map")
    p.add_argument("--robot-frame", default="base_link")
    p.add_argument("--lookahead", type=float, default=3.0,
                   help="furthest carrot distance, m")
    p.add_argument("--lookahead-min", type=float, default=1.5,
                   help="nearest carrot distance, m (clear of the footprint)")
    p.add_argument("--band", type=float, default=0.5,
                   help="distance-band width when sampling the boundaries, m")
    p.add_argument("--fov-deg", type=float, default=0.0,
                   help="camera horizontal field of view, degrees. Non-zero "
                        "enables the lane-side check in lane_side.py: with "
                        "only one line visible, each hypothesis about which "
                        "EDGE it is predicts where the other line should be, "
                        "and the field of view says whether that prediction "
                        "is falsifiable. 0.0 keeps the old assumption that "
                        "the lane is always on the far side of the line.")
    p.add_argument("--lane-width", type=float, default=3.0,
                   help="assumed corridor width when only one boundary is seen")
    p.add_argument("--side-gate", type=float, default=0.35,
                   help="ignore points closer than this to the centreline, m")
    p.add_argument("--half-width", type=float, default=0.33,
                   help="robot half width incl. margin, m (footprint is 0.31)")
    p.add_argument("--cone-radius", type=float, default=0.28,
                   help="keep-out radius around each cone point, m")
    p.add_argument("--blind-timeout", type=float, default=4.0,
                   help="give up and stop after this long with no lane paint "
                        "in view, s. Without it the blind corridor happily "
                        "drives straight off the course forever")
    p.add_argument("--blind-corridor", type=float, default=2.0,
                   help="half width of the corridor assumed drivable where "
                        "no paint is visible, m; obstacles still carve it. "
                        "0 disables it and the old blind coast returns")
    p.add_argument("--lane-memory", type=float, default=0.0,
                   help="keep steering along the last carrot that came from "
                        "real paint for this long after the paint is lost, s. "
                        "0 disables. Coasting STRAIGHT through a dropout is "
                        "only right on a straight; this finishes the bend it "
                        "was already on. It never extrapolates past the last "
                        "point actually seen")
    p.add_argument("--lidar-corridor", action="store_true",
                   help="when there is no paint, bound each band by the "
                        "nearest lidar return on each side and aim at the "
                        "middle of the widest gap -- Follow-the-Gap on the "
                        "raw scan. The barrels line the course, so this "
                        "steers with no appearance model at all")
    p.add_argument("--corridor-max", type=float, default=3.0,
                   help="half width searched by --lidar-corridor, m")
    p.add_argument("--max-lateral-step", type=float, default=0.6,
                   help="stop extending the carrot when the centreline "
                        "jumps more than this between bands, m")
    p.add_argument("--startup-grace", type=float, default=25.0,
                   help="before ANY real reading, allow this long, s. The "
                        "stack needs a moment; after the first reading the "
                        "tighter --blind-timeout applies")
    p.add_argument("--max-blind-dist", type=float, default=15.0,
                   help="stop after travelling this far with no lane "
                        "reading, m. The arena is 43 m across; one run "
                        "managed 216 m in a straight line out of it. "
                        "0 disables")
    p.add_argument("--idle-log", type=float, default=10.0,
                   help="while stopped, repeat the reason this often, s. "
                        "Silence must not be ambiguous between fine and dead")
    p.add_argument("--stuck-dist", type=float, default=0.25,
                   help="movement below this counts as not moving, m")
    p.add_argument("--stuck-secs", type=float, default=6.0,
                   help="after this long without moving, re-send the goal "
                        "even if it has not shifted. 0 disables")
    p.add_argument("--rate", type=float, default=2.0, help="Hz")
    p.add_argument("--min-goal-shift", type=float, default=0.75,
                   help="only republish once the carrot moves this far, m")
    p.add_argument("--cloud-timeout", type=float, default=1.5,
                   help="treat /lane/points as dead after this long, s")
    p.add_argument("--coast", action="store_true", default=True,
                   help="creep straight when no boundary is visible")
    p.add_argument("--no-coast", dest="coast", action="store_false")
    p.add_argument("--dry-run", action="store_true",
                   help="compute and log carrots, publish nothing")
    a = p.parse_args()

    rclpy.init()
    n = AutoDrive(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is the normal way this node ends: demo.sh
        # kills the process group, rclpy tears the context down itself, and
        # spin() raises. Catching only KeyboardInterrupt let it escape as a
        # traceback, and the rclpy.shutdown() below then failed a second time
        # with "rcl_shutdown already called on the given context" -- two
        # alarming stack traces for an ordinary Ctrl+C.
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
