# ESDA autonomous stack: what runs, and what each node does

How the pieces fit together on `yolo-lane-cone`, written while debugging the
lap. Accurate as of commit `a122fd5`.

The one-line version: **the navigator decides *where*, MPPI decides *how*.**
Nothing in our code commands a wheel.

## The chain

```
Gazebo ──camera──▶ lane_detection_esda ──/scan_fused──┬──▶ slam_toolbox ──/map──┐
   │                      │                           │                          │
   └──/scan───────────────┘                           └──▶ Nav2 costmaps         │
   │                      │                                                      │
   │                 /lane_markers                                               │
   │                      ▼                                                      │
   │          waypoint_navigator_recommendation ◀───────────────────────────────┘
   │                      │  goal pose, via the navigate_to_pose action
   │                      ▼
   │                    Nav2: planner → controller → /cmd_vel_nav
   └──────────────────────────────────────────────▶ /cmd_vel ──▶ wheels
```

Five processes, started in this order because each needs the previous one's
topics. `sweep/bench_run.sh` is the reference for how they are brought up.

## The nodes

### 1. Gazebo and the bridges — `launch_sim.launch.py`

Physics, and the sensor topics: `/camera/left/image_raw`,
`/camera/right/image_raw`, `/camera/depth/image_raw`, `/scan` from the lidar,
`/imu/data`. `gz_ros2_control` runs `diff_drive_base_controller`, which
consumes `/cmd_vel` and publishes wheel odometry on
`/diff_drive_base_controller/odom`. `robot_state_publisher` broadcasts TF from
the URDF.

### 2. The EKF — `robot_localization`, `config/ekf.yaml`

Fuses `/diff_drive_base_controller/odom` with `/imu/data` into
`/odometry/filtered`.

**`two_d_mode: true`**, which is why it reports exactly 0.00 deg of pitch
everywhere. It has no pitch to give, by construction. Do not try to get
attitude out of it.

### 3. `lane_detection_esda.py` — the detector

Our subclass of their detector. Takes the stereo pair, runs YOLOv8n-seg
(lane / cone / drivable), and publishes three things:

| topic | what it is | who consumes it |
|---|---|---|
| `/lane_markers` | `MarkerArray`, one cube per lane point, namespaced `left_lane` / `right_lane` | the navigator, to steer by |
| `/lane_obstacles` | `PointCloud2` of the same paint | the costmaps |
| `/scan_fused` | the lidar scan **with lane points injected into it** | SLAM and the costmaps |

That third one is the key non-standard move in the whole stack: painted lines
become laser returns, so everything downstream treats lane paint as a physical
obstacle. It is what keeps the robot in its lane without any lane-keeping
controller — and it is also why the corridor is tight (see the last section).

### 4. `slam_toolbox` — `online_async_launch.py`

Fed `/scan_fused`, not `/scan`. Builds `/map` and the `map -> odom` transform.
Because it eats the fused scan, the lane paint is baked into the map as walls.

### 5. Nav2 — `navigation_launch.py`

Under a lifecycle manager:

- **`planner_server`** — NavFn (`GridBased`) over the global costmap. The A\*
  that produces a route. The one that says `failed to create plan with
  tolerance 0.1`.
- **`controller_server`** — MPPI. Samples 1200 candidate trajectories x 40
  steps at 8 Hz, scores each against its critics (goal distance, path
  alignment, obstacle cost, `PreferForwardCritic` ...), publishes the winner.
  **This is the only thing that produces velocity.**
- **`bt_navigator`** — hosts the `navigate_to_pose` action and runs the
  behaviour tree that calls planner and controller and fires recoveries.
- **`behavior_server`** — holds `spin` and `backup`. Loaded at startup; in our
  runs, never actually invoked.
- **costmaps** — a 10x10 m rolling *local* one for the controller, a
  full-course *global* one for the planner. Both take `/scan_fused` plus
  `/lane_obstacles`, mark obstacles and inflate them (0.6 m local, 0.8 m
  global).
- **`velocity_smoother`** — `/cmd_vel_nav` -> `/cmd_vel`.

Note `collision_monitor` has a parameter block in `nav2_params.yaml` but is
**not launched**, so nothing is doing last-resort collision braking.

### 6. `waypoint_navigator_recommendation.py` — the driver

The brain, and the biggest piece at ~4500 lines. Despite the name it is **not**
a waypoint-list follower: it generates goals continuously.

**Normal driving.** Each cycle it reads `/lane_markers`, transforms the points
into the map frame, drops any sitting on a lidar return (barrel stripes look
like paint to the detector), pairs the boundaries nearest-first and averages
them into a **centreline**, then projects a goal ~1.5 m along it and sends it
with `BasicNavigator.goToPose()`. There is no fixed route anywhere.

**Four waypoints on top.** `waypoints/igvc_waypoints.yaml` holds
`no_mans_land_start`, `ramp_start`, `ramp_end`, `no_mans_land_end` along the
top straight. When the next is within `gps_distance_threshold` it becomes the
Nav2 goal and the node switches to `mode=gps`. They exist because the lanes
genuinely stop there — barricade, ramp, barricade — so lane following has
nothing to work with.

**A recovery ladder** for when Nav2 fails: `find_front_recovery_goal`, then
`find_far_recovery_goal`, then `find_left_recovery_goal` and
`find_stuck_nudge_goal`; `schedule_recovery_retry` retries at 1 Hz, and
`handle_no_candidate_timeout` releases the lock after 20 s of finding nothing.
While in recovery it **blocks** normal goal generation.

**Clearance maths** as module-level pure functions over distance transforms of
the occupancy grid, so they can be exercised without rclpy (see
`tools/validate_waypoint_logic.py`). `map_clearance` blends two transforms:
`min(dt_hard, dt_unknown + 0.8)` — so unobserved space counts as nearly an
obstacle, which is why recovery struggles in open, sparsely mapped worlds.

**A keepout trail** — a breadcrumb every 5 m, published as a costmap keepout
mask (radius 0.8 m) on `/keepout_costmap_filter_info`, discouraging Nav2 from
re-driving where it has been.

Modes: `normal`, `gps`, `recovery`, `backup`.

> Gotcha: it loads the waypoint YAML from the **relative** path
> `src/esda_simulation_2025/waypoints/igvc_waypoints.yaml`, so it only finds
> its waypoints when launched with the workspace as the working directory.
> `bench_run.sh` happens to `cd` there; a plain `ros2 run` elsewhere will not.

### 7. `research/vision/lap_test.py` — scoring only

Not in the control loop. It counts goals on `/goal_pose`, which the navigator
never uses (it uses the action), so `goals published` always reads 0.

## Not in the loop

`for_server/vision/auto_drive.py` is a second, independent driver: a direct
lane follower that computes a steering carrot and publishes `cmd_vel` itself,
bypassing Nav2 entirely. `lane_side.py` came from there. `bench_run.sh` runs
the Nav2 path, not this one.

## The thing worth understanding

Injecting lane paint into `/scan_fused` makes the lines obstacles in the
costmap *and* walls in the SLAM map. Two painted lines 3 m apart, each inflated
0.8 m globally, leave about 1.4 m of corridor for a robot Nav2 models as a
0.9 m circle — roughly 0.25 m of margin per side. Most of the behaviour that
looks like a planner or controller problem lives in that margin.

Related, and unresolved: `description/robot_core_ref.xacro` puts `base_link` at
the axle with the chassis running **back** to -0.965, so the 0.45 m circle is
0.14 m too wide on each side and leaves 0.52 m of rear overhang unmodelled.
That now matters, because the robot can reverse. A polygon footprint was tried
and reverted — its circumscribed radius (1.07 m) exceeds both
`inflation_radius` values, which Nav2 cannot represent, and raising inflation
to match would take 2.14 m out of a 3.0 m lane.
