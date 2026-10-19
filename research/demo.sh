#!/usr/bin/env bash
# =====================================================================
#  ESDA demo driver — bring the whole stack up in the right order,
#  send goals, record, tear down.
#
#      ./demo.sh up              sim + SLAM + Nav2 + RViz  (software GL)
#      ./demo.sh up --vision     ...and the YOLO lane/cone node, with the
#                                vision costmap layers enabled
#      ./demo.sh up --world igvc_oval
#                                load a generated arena instead of the stock
#                                course. vision/make_worlds.py builds
#                                igvc_oval, igvc_wavy, igvc_figure8,
#                                igvc_slalom and igvc_spline -- driving those
#                                and capturing frames is how you get real
#                                ogre2 training data from layouts the model
#                                has not seen.
#      ./demo.sh up --no-rviz    skip RViz. It has crashed repeatedly here and
#                                is a second software-GL renderer competing
#                                for the CPU; use ./demo.sh gui --sw instead.
#      ./demo.sh up --gpu        render Gazebo + RViz on the GPU instead of
#                                llvmpipe. This is the single biggest CPU
#                                saving available: with software GL every
#                                camera and lidar frame is rasterised on the
#                                CPU. Flags combine: up --gpu --vision
#                                Run `bash gpu_check.sh` first to see whether
#                                hardware GL actually works on this box.
#      ./demo.sh goal 3 0        drive to (x=3, y=0) in the map frame
#      ./demo.sh drive           FULLY AUTONOMOUS. No goal, no waypoints:
#                                steers from the lane boundaries the camera
#                                sees and keeps going. Needs --vision.
#                                ./demo.sh drive --lookahead 3.0
#      ./demo.sh auto            drive a course automatically: a list of
#                                waypoints via /follow_waypoints. Defaults to
#                                4,0 8,0 12,0 -- straight ahead 12 m, which
#                                stops short of the barrel ~14.4 m along +x.
#                                ./demo.sh auto 5,0 12,0 20,0 --laps 2
#                                Waypoints are MAP frame, i.e. relative to
#                                where the robot started.
#      ./demo.sh up --spawn -11,31,0
#                                put the robot somewhere specific. The stock
#                                igvc world defaults to this already: the west
#                                end of its 25 m barrel corridor.
#      ./demo.sh lap igvc_oval_test --seconds 300
#                                ONE COMMAND, UNATTENDED. Brings the stack up
#                                on a held-out world, drives with no goal,
#                                scores whether it got round, tears down, and
#                                writes logs/run_<world>_<stamp>/ with the
#                                report and every stage log. Flags after a
#                                bare -- go to the driver:
#                                ./demo.sh lap igvc_wavy_test -- --lookahead 2.5
#      ./demo.sh record          start a rosbag + an mp4 of the debug image
#      ./demo.sh stoprec         stop recording
#      ./demo.sh gui             open a Gazebo GUI onto the running sim
#                                (add --sw if the window is black)
#      ./demo.sh rviz            open RViz on the running stack (separate
#                                process; if it crashes nothing else does)
#      ./demo.sh logs            copy all stage logs into ./logs/
#      ./demo.sh status          what is running / key topics
#      ./demo.sh teleop          drive by hand (overrides Nav2 via twist_mux)
#      ./demo.sh down            kill everything
#
#  Each stage WAITS for the previous one to actually come up instead of
#  sleeping and hoping. That matters here: Nav2 launched before /clock and
#  the controllers exist will activate against a missing TF tree and then sit
#  there refusing goals, which looks like "Nav2 is broken" when it is only
#  "Nav2 started too early".
# =====================================================================
set -uo pipefail

# All the launch/teardown machinery lives in esda_lib.sh so demo.sh, run.sh and
# anything else share ONE implementation. In particular teardown is pid-based:
# see the header of esda_lib.sh for why `pkill -f <bare word>` was a hazard.
# shellcheck source=esda_lib.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/esda_lib.sh"

cmd_up() {
  local vision=0
  local WORLD="" SPAWN="" RVIZ=true show=0 vision_args=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --vision) vision=1 ;;
      # Lane only: stop the vision node publishing cones and let the LIDAR
      # own obstacles. It already does -- /scan feeds the VoxelLayer locally
      # and the ObstacleLayer globally -- and cone_gate already threw away
      # any vision cone no lidar return confirmed, so the system has been
      # treating vision cones as a guess and lidar as the measurement all
      # along. This deletes the guess. The barrels on these courses are
      # construction_barrel.dae at roughly 0.9 m against a lidar ring at
      # 0.5475 m, so the level ring sees all of them; anything shorter than
      # the ring height would need vertical rows put back in lidar.xacro.
      #
      # NOTE this does not speed up inference. The model still computes all
      # three heads; mode only mutes the output. The speed win needs a
      # retrained single-class model, which is also the better model.
      --lane-only) vision=1; vision_args="$vision_args --mode lane_only" ;;
      # Pick the detector without editing vision_config.yaml, so A/B-ing
      # otsu against adaptive is a flag rather than a file edit.
      --detector) shift; vision=1
                  vision_args="$vision_args --detector ${1:-}" ;;
      --method)   shift; vision=1
                  vision_args="$vision_args --detector classic --classic-method ${1:-}" ;;
      # Costmaps published, RViz left on: for a run being watched or recorded.
      --show)   vision=1; show=1; RVIZ=true ;;
      --gpu)    GPU_RENDER=1 ;;
      # ogre1 instead of ogre2. ogre2 is a PBR renderer -- shadow maps, HDR,
      # a compositor chain -- and under WSL all of it runs in software on
      # llvmpipe, competing with Nav2 for the same cores. The camera is the
      # whole budget here, so a cheaper renderer is worth measuring.
      --engine) shift; ESDA_RENDER_ARGS="--render-engine ${1:-ogre}"
                export ESDA_RENDER_ARGS ;;
      --world)  shift; WORLD="${1:-}"
                [ -n "$WORLD" ] || { echo "--world needs a name"; exit 1; } ;;
      --spawn)  shift; SPAWN="${1:-}"
                [ -n "$SPAWN" ] || { echo "--spawn needs x,y[,yaw]"; exit 1; } ;;
      --no-rviz) RVIZ=false ;;
      *) echo "unknown flag for up: $1"; exit 1 ;;
    esac
    shift
  done
  src

  # Where to put the robot.
  #
  # The generated arenas keep a 3 m clear radius at the origin, so 0,0 suits
  # them. For the stock igvc world the start line was established empirically:
  # a marker sphere was dropped in the Gazebo GUI at the point on the course
  # where the run should begin, and `ign model -m sphere -p` reported
  #     XYZ [0.450747 0.025746 0.500000]
  # i.e. essentially the world origin, between the two painted lines.
  #
  # Worth recording why this needed measuring. The arena model is included at
  # <pose>0 16.3 0</pose>, so reading barrel coordinates out of model.sdf and
  # adding the offset suggests the course sits around world y = 31 -- which is
  # where an earlier version of this spawned the robot, well off the lane. The
  # painted course and the barrel field are not the same thing: the barrels in
  # that model mark the arena's outer edges, not the driving line. Along
  # y = 0 the nearest ones are at world x = -19.2, -16.1, +14.9, so facing +x
  # there is a clear ~14 m run before the first obstacle.
  local SX=0.0 SY=0.0 SYAW=0.0
  if [ -n "$SPAWN" ]; then
    IFS=, read -r SX SY SYAW <<<"$SPAWN"
    SYAW="${SYAW:-0.0}"
  else
    # The spawn table moved to config/world_spawns.txt so that the capture
    # tool reads the same numbers -- see world_spawn() in esda_lib.sh for why.
    local _sp; _sp="$(world_spawn "${WORLD:-igvc}")"
    [ -n "$_sp" ] && read -r SX SY SYAW <<<"$_sp"
  fi

  if [ "$GPU_RENDER" = "1" ]; then
    say "0/4  GPU rendering requested -- checking OpenGL"
    local r
    r="$(LIBGL_ALWAYS_SOFTWARE=0 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
         glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
    if [ -z "$r" ]; then
      warn "glxinfo unavailable (sudo apt install -y mesa-utils). Trying the GPU anyway."
    elif echo "$r" | grep -qiE "llvmpipe|softpipe|swrast"; then
      warn "hardware GL resolves to a SOFTWARE renderer: $r"
      warn "--gpu will buy you nothing. Run: bash gpu_check.sh"
    else
      ok "GPU renderer: $r"
    fi
  fi
  [ -d "$WS/install/$PKG" ] || die "workspace not built. Run: bash setup_wsl.sh"

  local variant=""
  [ "$vision" = "1" ] && variant="vision"
  [ "$show" = "1" ] && variant="show"
  local params_path; params_path="$(nav2_params_path "$variant")"
  [ -f "$params_path" ] || die "missing $params_path (re-run: bash setup_wsl.sh --sync)"

  say "1/4  Gazebo (headless, world: ${WORLD:-igvc}, spawn $SX,$SY yaw $SYAW, rviz=$RVIZ) + controllers + twist_mux"
  start_sim "$WORLD" "$SX" "$SY" "$SYAW" "$RVIZ"
  if ! wait_for "/clock (sim is ticking)"  90 has_data /clock; then
    if [ "$GPU_RENDER" = "1" ]; then
      die "Gazebo never started WITH --gpu. ogre2 on WSL hardware GL is the
    usual culprit. Retry without it:   ./demo.sh down && ./demo.sh up
    Log: $LOGS/sim.log"
    fi
    die "Gazebo never started -- see $LOGS/sim.log"
  fi
  if ! wait_for "/scan (lidar)"            90 has_data /scan; then
    warn "no /scan yet; SLAM will not work"
    [ "$GPU_RENDER" = "1" ] && warn "with --gpu this often means the ogre2 sensor
        render died. Check $LOGS/sim.log, and fall back to software GL."
  fi
  # 150s, not 60s: ign_ros2_control has to fetch the URDF from
  # robot_state_publisher over a service, register the wheel joints and let
  # the spawner configure+activate the controller. On a cold WSL start that
  # measured over 60s here while everything was in fact healthy, so the old
  # budget produced a scary warning about a stack that was working.
  wait_for "diff_drive controller listening" 150 has_sub /diff_drive_base_controller/cmd_vel_unstamped \
    || warn "the drive controller is not subscribed -- check $LOGS/sim.log"
  wait_for "twist_mux (Nav2 -> wheels)"     30 has_node twist_mux \
    || warn "twist_mux missing: Nav2 will plan but the robot will NOT move"

  say "2/4  SLAM (slam_toolbox, mapping mode)"
  start_slam
  wait_for "/map"                           90 has_pub /map          || warn "no /map -- Nav2's global costmap will be empty"

  say "3/4  Nav2   (params: $(basename "$params_path"))"
  start_nav2 "$variant"
  # The flag is what cmd_lap reads. Three of the twenty-three runs in logs/
  # had Nav2 stuck in lifecycle -- 676 repeats of "Waiting for service
  # bt_navigator/get_state" in one of them -- and every one was written up as
  # "FAIL - no lap completed, path length 0.0 m", which reads exactly like a
  # driving result and is not one. A run with no controller measures nothing.
  rm -f "$LOGS/nav2_active.flag"
  if wait_for "bt_navigator active"         120 node_active /bt_navigator; then
    echo yes > "$LOGS/nav2_active.flag"
    # Now that it is up, push Nav2 down the scheduler.
    #
    # Carrots published == goals accepted == lane publishes, exactly (42/42,
    # 20/20, 22/22 across three runs), so distance travelled is set by how
    # many camera frames get through -- and the camera renders on llvmpipe,
    # in software, on the same cores MPPI is rolling out trajectories on.
    # With Nav2 dead one run saw 379 lane publishes; with Nav2 alive the same
    # world gives 15-20. They are competing, and the renderer should win.
    #
    # Deliberately AFTER activation, not before: lifecycle and bond timeouts
    # are wall-clock bound, so starting Nav2 niced would risk the very
    # failure the flag above exists to catch. renice upward needs no
    # privilege, and bg() uses setsid, so the saved pid leads the group.
    if [ -f "$LOGS/nav2.pid" ]; then
      renice -n 10 -g "$(cat "$LOGS/nav2.pid")" >/dev/null 2>&1 \
        && ok "Nav2 reniced to 10 -- the Gazebo render thread now wins ties" \
        || warn "could not renice Nav2 (harmless; it just keeps equal priority)"
    fi
  else
    echo no > "$LOGS/nav2_active.flag"
    warn "Nav2 did not reach active -- see $LOGS/nav2.log"
  fi

  if [ "$vision" = "1" ]; then
    say "4/4  Vision node (YOLO lane/cone + linking)"
    local w="$HOME/esda_models/lane_seg.pt"
    [ -f "$w" ] || warn "no weights at $w -- the node runs in pass-through mode"
    bg vision "python3 $ESDA_SRC/vision/lane_seg_node.py $vision_args"
    wait_for "/lane/debug_image"            60 has_pub /lane/debug_image || warn "vision node not publishing"
  else
    say "4/4  (vision skipped -- use ./demo.sh up --vision to include it)"
  fi

  cat <<EOF

--------------------------------------------------------------------
STACK UP.  In the RViz window:
  * set Fixed Frame to 'map'
  * click '2D Goal Pose' and click a spot ahead of the robot
    ...or from a terminal:   ./demo.sh goal 3 0

Drive by hand instead:       ./demo.sh teleop
Record the run:              ./demo.sh record
What is running:             ./demo.sh status
Stop everything:             ./demo.sh down

Logs: $LOGS/{sim,slam,nav2,vision}.log
--------------------------------------------------------------------
EOF
}

cmd_show() {
  # Everything up, everything visible, driving -- for recording.
  #
  #     ./demo.sh show                       (igvc_oval_slow_test)
  #     ./demo.sh show igvc
  #     ./demo.sh show igvc --method otsu
  #     ./demo.sh show igvc -- --lookahead 3.5
  #
  # Three renderers end up competing for the same cores here: Gazebo's
  # offscreen camera, the Gazebo GUI and RViz, all on llvmpipe. So this will
  # drive WORSE than the benchmark numbers, by design. It is for looking at,
  # not for measuring -- use ./demo.sh lap for that.
  local world="igvc_oval_slow_test"
  local -a driveflags=()
  local -a upflags=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --)  shift; driveflags=("$@"); break ;;
      --method|--detector) upflags+=("$1" "${2:-}"); shift ;;
      --lane-only) upflags+=("$1") ;;
      -*)  echo "unknown flag for show: $1"; exit 1 ;;
      *)   world="$1" ;;
    esac
    shift
  done
  [ "${#driveflags[@]}" -eq 0 ] && \
    driveflags=(--lane-memory 6.0 --lidar-corridor --blind-timeout 8)

  say "SHOW  world=$world  -- costmaps published, RViz + Gazebo GUI + camera"
  warn "this is for watching, not measuring: the GUI and RViz take CPU
    off the same software renderer the robot's camera uses, so it will
    drive worse here than ./demo.sh lap reports."

  stop_all >/dev/null 2>&1
  sleep 2
  cmd_up --show --world "$world" "${upflags[@]}"
  if [ "$(cat "$LOGS/nav2_active.flag" 2>/dev/null)" != "yes" ]; then
    warn "Nav2 never activated -- retrying the bring-up once"
    stop_all >/dev/null 2>&1; sleep 5
    cmd_up --show --world "$world" "${upflags[@]}"
  fi
  if [ "$(cat "$LOGS/nav2_active.flag" 2>/dev/null)" != "yes" ]; then
    # die() does not tear anything down, so Gazebo, RViz and the detector all
    # stay up and LOOK healthy -- which is exactly how somebody ends up
    # staring at three live windows wondering why the robot will not move.
    # Say what is actually wrong and what can still be done.
    warn "Nav2 is NOT active, so NOTHING WILL DRIVE."
    cat <<EOF

    The stack is still up -- Gazebo, RViz and the lane detector are all
    running, which is why the windows look fine. But auto_drive publishes
    goals to /goal_pose and bt_navigator is what executes them. With no
    bt_navigator there is nobody to drive, so the driver was not started.

    You can still:
      ./demo.sh teleop    drive it by hand and watch the detector work
      ./demo.sh status    see which nodes are alive
      ./demo.sh down      stop everything

    To find out why Nav2 refused, send me the log:
      cp $LOGS/nav2.log $ESDA_SRC/logs/nav2_fail.log

EOF
    die "Nav2 will not activate -- see $LOGS/nav2.log"
  fi

  say "Attaching the Gazebo window"
  start_gui
  sleep 6

  cmd_drive "${driveflags[@]}"

  cat <<EOF

--------------------------------------------------------------------
RECORDING SETUP IS UP.  Three windows:

  Gazebo     third-person view of the robot on the course
  RViz       local + global costmaps, the lane cloud (green), barrels
             (orange), the fitted centreline, the magenta carrot, and
             the detector's overlay in the Image panel
  (RViz)     'Lane detector overlay' shows the camera frame WITH the
             model's masks drawn -- that is the view worth recording

In RViz, set Fixed Frame to 'map' if it is not already.

Watch it think:   tail -f $LOGS/drive.log
Perception rate:  grep perception: $LOGS/vision.log | tail
Stop everything:  ./demo.sh down
--------------------------------------------------------------------
EOF
}

cmd_goal() {
  local x="${1:-2.0}" y="${2:-0.0}"
  src
  say "Sending NavigateToPose goal  x=$x  y=$y  (frame: map)"
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, \
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" --feedback
}

cmd_teleop() {
  src
  say "Teleop on /cmd_vel_joy (twist_mux priority 100 -- overrides Nav2)"
  echo "    i/j/k/l to drive, q/z to change speed, Ctrl+C to hand control back"
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_joy
}

cmd_record() {
  src
  local ts; ts="$(date +%Y%m%d_%H%M%S)"
  local dir="$RUNS/$ts"; mkdir -p "$dir"
  say "Recording to $dir"
  bg bag "cd $dir && ros2 bag record -o bag \
      /tf /tf_static /scan /odom /map /cmd_vel /plan \
      /diff_drive_base_controller/cmd_vel_unstamped \
      /lane/points /lane/points_raw /cone/points /lane/lines \
      /local_costmap/costmap /global_costmap/costmap"
  if src && ros2 topic list 2>/dev/null | grep -qx /lane/debug_image; then
    bg vid "cd $dir && ros2 run image_view video_recorder \
        --ros-args -r image:=/lane/debug_image -p filename:=$dir/debug.avi"
    echo "    also recording /lane/debug_image -> $dir/debug.avi"
  fi
  echo "$dir" > "$RUNS/.current"
  cat <<EOF

    rosbag + debug video are running.  Stop with:  ./demo.sh stoprec

    For a SCREEN recording of the RViz window (the thing worth showing),
    use Windows directly -- WSLg windows are ordinary Windows windows:
      * Xbox Game Bar:  Win+G, or Win+Alt+R to start/stop
      * or OBS Studio -> Window Capture -> the RViz window
    Record the screen, not the bag, if you want something to show people.
EOF
}

cmd_stoprec() {
  say "Stopping recorders"
  pkill -INT -f "ros2 bag record" 2>/dev/null
  pkill -INT -f "image_view/video_recorder" 2>/dev/null
  sleep 2
  local d; d="$(cat "$RUNS/.current" 2>/dev/null || true)"
  [ -n "$d" ] && { echo "    saved in $d"; ls -la "$d" 2>/dev/null; }
}

cmd_drive() {
  # Fully autonomous: no goal, no waypoints. auto_drive.py reads the lane
  # boundaries, aims a carrot down the centreline and keeps republishing it,
  # so the robot just goes. Stop with ./demo.sh down.
  src
  # Wait rather than refuse. A single `ros2 topic list` right after another
  # command can miss a topic that exists: ROS 2 discovery is not instant, and
  # this check fired a false "vision node is not running" when the node was
  # up and publishing. Only give up after 20s, and even then start the driver
  # -- it idles harmlessly until points arrive, which beats refusing to run.
  if ! wait_for "/lane/points" 20 has_topic /lane/points; then
    warn "/lane/points has not appeared in 20s."
    warn "If the stack was started WITHOUT --vision, that is why:"
    warn "    ./demo.sh down && ./demo.sh up --vision"
    warn "Starting the driver anyway -- it will idle until points arrive."
  fi
  say "Autonomous lane following -- no waypoints"
  start_drive "$*"
  sleep 2
  echo "    watch it think:   tail -f $LOGS/drive.log"
  echo "    stop:             ./demo.sh down"
}

cmd_auto() {
  # Drive a sequence of waypoints without further input -- the "lap the
  # course" mode, as opposed to a single `goal`.
  #
  # Uses /follow_waypoints (nav2_waypoint_follower), NOT navigate_through
  # _poses: follow_waypoints drives to each pose in turn and replans for each
  # one, so a barrel appearing mid-lap is handled per leg. navigate_through
  # _poses plans ONE path through every pose up front, which is the wrong
  # shape for a course run.
  #
  # Waypoints are in the MAP frame. slam_toolbox puts the map origin at the
  # robot's pose when SLAM started, which is NOT the Gazebo world origin --
  # so "5,0" means 5 m ahead of wherever the robot began. That is why one
  # default list works in every arena.
  local reps=1 wps=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --laps) shift; reps="${1:-1}" ;;
      "")     ;;
      *)      wps="$wps $1" ;;
    esac
    shift
  done
  wps="$(echo "$wps" | xargs)"
  [ -z "$wps" ] && wps="4,0 8,0 12,0"
  src

  local poses="" p x y i=0
  while [ "$i" -lt "$reps" ]; do
    for p in $wps; do
      x="${p%%,*}"; y="${p##*,}"
      poses="$poses{header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {w: 1.0}}}, "
    done
    i=$((i+1))
  done
  poses="${poses%, }"

  say "Autonomous run: $(echo "$wps" | wc -w) waypoints x $reps lap(s), map frame"
  echo "    $wps"
  echo "    (map frame = relative to where the robot started, not the world origin)"
  ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
    "{poses: [$poses]}" --feedback
}

cmd_gui() {
  # Hardware GL by default: glxinfo reported D3D12 on the RTX 5060 Ti here, and
  # the GUI is the one thing worth spending the GPU on. --sw falls back to
  # llvmpipe if the window comes up black or the driver gives up.
  case "${1:-}" in --sw) GPU_RENDER=0 ;; *) GPU_RENDER=1 ;; esac
  src
  say "Attaching a Gazebo GUI to the running server"
  start_gui
  cat <<EOF
    A window should appear within a few seconds showing the live simulation.
    It is a VIEWER on the running server -- closing it does not stop the sim.

    Black window, or it dies on startup? Retry on software GL:
        pkill -f "ign gazebo -g" ; ./demo.sh gui --sw
EOF
}

cmd_rviz() {
  # RViz on demand. It is no longer part of the sim launch (it crashed twice
  # and is a second software-GL renderer), but it is still the only way to see
  # costmaps, the global plan and the lane pointclouds overlaid on the map.
  # Started as its own stage, so ./demo.sh down cleans it up and its dying
  # takes nothing else with it.
  src
  say "RViz (separate from the sim -- if it crashes, nothing else does)"
  start_rviz
  echo "    Fixed Frame is 'map'. Useful displays already in the config:"
  echo "      Map (SLAM), Local/Global Costmap, Plan, Local Plan, LaserScan, Camera"
}

cmd_lap() {
  # ONE command, unattended: bring the stack up on a world, drive with no
  # goal, score the run, tear down, leave a report behind.
  #
  #     ./demo.sh lap igvc_oval_test --seconds 300
  #     ./demo.sh lap igvc_wavy_test --seconds 240 -- --lookahead 2.5
  #
  # Everything after a bare -- is handed to vision/auto_drive.py, which is
  # how a parameter is changed between runs without editing anything.
  #
  # Each run gets its own folder under logs/ holding the JSON report, the
  # human summary, and the stage logs, so a run can be read afterwards by
  # someone who was not watching it -- which is the whole point.
  local world="igvc_oval_test" secs=300 minturn="" minlap="" note=""
  local -a upflags=() driveflags=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --seconds)  shift; secs="${1:-300}" ;;
      --note)     shift; note="${1:-}" ;;
      --min-turn) shift; minturn="${1:-}" ;;
      --min-lap)  shift; minlap="${1:-}" ;;
      --gpu)      upflags+=(--gpu) ;;
      --engine)   shift; upflags+=(--engine "${1:-ogre}") ;;
      --lane-only) upflags+=(--lane-only) ;;
      --)         shift; driveflags=("$@"); break ;;
      -*)         echo "unknown flag for lap: $1"; exit 1 ;;
      *)          world="$1" ;;
    esac
    shift
  done

  # A figure of eight nets zero heading change over one circuit when the
  # start is not the crossing point, so the turn term would reject a lap it
  # actually drove. See the header of vision/lap_test.py.
  if [ -z "$minturn" ]; then
    case "$world" in *figure8*) minturn=0 ;; *) minturn=5.5 ;; esac
  fi
  # The generated arenas are roughly 30 m across, so a lap of one is 80-90 m.
  # 50 m is comfortably below a real lap and far above any wobble.
  [ -z "$minlap" ] && minlap=50

  local stamp rundir
  stamp="$(date +%Y%m%d_%H%M%S)"
  rundir="$SCRIPT_DIR/logs/run_${world}_${stamp}"
  mkdir -p "$rundir"

  say "LAP BENCHMARK  world=$world  limit=${secs}s  lap>=${minlap}m  min-turn=$minturn"
  echo "    engine  -> ${ESDA_RENDER_ARGS:-ogre2 (default)}"
  echo "    report -> $rundir"
  [ "${#driveflags[@]}" -gt 0 ] && echo "    driver  -> ${driveflags[*]}"
  case "$world" in
    *_test) ok "held-out world -- not one the parameters were tuned on" ;;
    *)      warn "$world is a world the parameters have been tuned against;
    the number it produces is optimistic. The held-out set is
    igvc_oval_test, igvc_wavy_test, igvc_figure8_test, igvc_spline_test." ;;
  esac

  stop_all >/dev/null 2>&1
  sleep 2
  cmd_up --vision --no-rviz --world "$world" "${upflags[@]+"${upflags[@]}"}"

  # A dead Nav2 is a broken rig, not a slow robot. Retry the bring-up once --
  # the failure is intermittent (3 of 23) and a retry costs two minutes where
  # scoring a dead stack costs the full seven and teaches nothing.
  if [ "$(cat "$LOGS/nav2_active.flag" 2>/dev/null)" != "yes" ]; then
    warn "Nav2 never activated -- tearing down and bringing the stack up once more"
    stop_all >/dev/null 2>&1
    sleep 5
    cmd_up --vision --no-rviz --world "$world" "${upflags[@]+"${upflags[@]}"}"
  fi
  if [ "$(cat "$LOGS/nav2_active.flag" 2>/dev/null)" != "yes" ]; then
    {
      echo "LAP TEST  $world"
      echo "$(date '+%F %T')  -- ${note:-no note}"
      echo
      echo "  SETUP FAILED - Nav2 never reached active. NOT a driving result."
      echo
      echo "  Nothing was measured: with no controller server the robot cannot"
      echo "  move, so any distance, speed or lap count here would be a"
      echo "  property of the rig and not of the navigation. The bring-up was"
      echo "  retried once and failed twice."
      echo
      echo "  Look for 'Waiting for service bt_navigator/get_state' in nav2.log."
      echo "  grep -c it: $(grep -c 'Waiting for service' "$LOGS/nav2.log" 2>/dev/null || echo '?')"
    } | tee "$rundir/lap_SETUP_FAILED.txt"
    printf '{"setup_ok": false, "world": "%s", "reason": "nav2 never active"}\n' \
      "$world" > "$rundir/lap_setup_failed.json"
    stop_all >/dev/null 2>&1
    cp "$LOGS"/*.log "$rundir/" 2>/dev/null
    warn "Report: $rundir/lap_SETUP_FAILED.txt"
    return 1
  fi

  cmd_drive "${driveflags[@]+"${driveflags[@]}"}"

  say "Scoring for up to ${secs}s -- nothing to do until it finishes"
  src
  python3 "$ESDA_SRC/vision/lap_test.py" \
      --world "$world" --seconds "$secs" --min-turn "$minturn" \
      --min-lap "$minlap" \
      --note "$note" --out "$rundir" 2>&1 | tee "$rundir/lap_test.log"

  stop_all >/dev/null 2>&1
  cp "$LOGS"/*.log "$rundir/" 2>/dev/null
  cp "$rundir/lap_latest.txt" "$SCRIPT_DIR/logs/lap_latest.txt" 2>/dev/null
  say "Done. Report and stage logs: $rundir"
  ls -1 "$rundir"
}

cmd_logs() {
  # Collect the stage logs into the repo folder, where they are visible from
  # Windows (and to anyone helping remotely) instead of buried in ~/esda_logs.
  local d="$SCRIPT_DIR/logs"
  mkdir -p "$d"
  cp "$LOGS"/*.log "$d/" 2>/dev/null
  say "Logs copied to $d"
  ls -la "$d"/*.log 2>/dev/null | awk '{printf "    %8s  %s\n", $5, $9}'
}

cmd_status() { stack_status; }

cmd_down() { stop_all; echo "    check with: ./demo.sh status"; }

case "${1:-}" in
  up)      shift; cmd_up "$@" ;;
  goal)    shift; cmd_goal "${1:-2.0}" "${2:-0.0}" ;;
  teleop)  cmd_teleop ;;
  record)  cmd_record ;;
  stoprec) cmd_stoprec ;;
  drive)   shift; cmd_drive "$@" ;;
  auto)    shift; cmd_auto "$@" ;;
  rviz)    cmd_rviz ;;
  show)    shift; cmd_show "$@" ;;
  lap)     shift; cmd_lap "$@" ;;
  logs)    cmd_logs ;;
  gui)     shift; cmd_gui "${1:-}" ;;
  status)  cmd_status ;;
  down)    cmd_down ;;
  *) sed -n '2,20p' "$0"; exit 1 ;;
esac
