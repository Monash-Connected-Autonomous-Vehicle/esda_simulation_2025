#!/usr/bin/env bash
# demo.sh -- bring the ESDA simulation up in the order that actually works.
#
#   ./demo.sh up            everything, with health gates between stages
#   ./demo.sh up --no-nav   sim + lane detection + SLAM + RViz only
#   ./demo.sh status        what is running, and whether the topics are alive
#   ./demo.sh down          kill everything
#   ./demo.sh goal X Y      send a Nav2 goal in the map frame
#   ./demo.sh logs NAME     tail one stage's log (SIM LANE SLAM NAV RVIZ)
#
# Options for `up`:
#   --world NAME     default igvc.sdf
#   --method NAME    yolo (default) | adaptive | fixed | otsu | tophat | value
#   --no-lane        skip lane detection  (also switches the stack to /scan)
#   --no-nav         skip Nav2
#   --no-rviz        skip RViz
#   --no-build       skip the colcon build
#
# Deliberately NOT `set -e`: a failed health check should report and stop
# cleanly, not kill the shell mid-launch.
set -o pipefail

WS="${WS:-$HOME/esda_ws}"
PKG="esda_simulation_2025"
LOGDIR="$WS/.demo_logs"
PIDDIR="$WS/.demo_pids"

WORLD="igvc.sdf"
METHOD="yolo"
DO_LANE=1; DO_NAV=1; DO_RVIZ=1; DO_BUILD=1; DO_DRIVE=0

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_0=$'\033[0m'
say()  { printf '%s\n' "$*"; }
step() { printf '\n%s==> %s%s\n' "$C_DIM" "$*" "$C_0"; }
ok()   { printf '    %sOK%s   %s\n' "$C_OK" "$C_0" "$*"; }
warn() { printf '    %swarn%s %s\n' "$C_WARN" "$C_0" "$*"; }
die()  { printf '\n%sFAILED:%s %s\n\n' "$C_ERR" "$C_0" "$*" >&2; exit 1; }

# ---------------------------------------------------------------- environment
setup_env() {
  [ -d "$WS" ] || die "no workspace at $WS (override with WS=/path ./demo.sh ...)"
  mkdir -p "$LOGDIR" "$PIDDIR"
  # shellcheck disable=SC1091
  [ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble not found at /opt/ros/humble"
  source /opt/ros/humble/setup.bash || die "sourcing /opt/ros/humble/setup.bash failed"
  if [ ! -f "$WS/install/setup.bash" ]; then
    if [ "${ACTION:-}" = "up" ]; then
      step "First run: $WS has never been built. Building now (a few minutes)."
      mkdir -p "$LOGDIR"
      ( cd "$WS" && colcon build --symlink-install ) 2>&1 | tee "$LOGDIR/BUILD.log" | tail -5
      [ -f "$WS/install/setup.bash" ] || die "initial build failed -- see $LOGDIR/BUILD.log"
      DO_BUILD=0
      ok "initial build complete"
    else
      die "$WS is not built yet -- run ./demo.sh up first"
    fi
  fi
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  local prefix; prefix="$(ros2 pkg prefix "$PKG" 2>/dev/null)"
  case "$prefix" in
    "$WS"/*) : ;;
    "")      die "package $PKG not found after sourcing $WS" ;;
    *)       die "wrong workspace active: $prefix
       Your ~/.bashrc is sourcing a different one. Open a clean shell, or fix .bashrc." ;;
  esac
  mkdir -p "$LOGDIR" "$PIDDIR"
}

# ------------------------------------------------------------------- helpers
start() {                     # start NAME "command"
  local name="$1"; shift
  local log="$LOGDIR/$name.log"
  : > "$log"
  setsid bash -c "set +u; source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; cd '$WS'; exec $*" \
    >>"$log" 2>&1 &
  echo $! > "$PIDDIR/$name.pid"
  ok "$name started (pid $!, log: $log)"
}

alive() { local p; p="$(cat "$PIDDIR/$1.pid" 2>/dev/null)"; [ -n "$p" ] && kill -0 "$p" 2>/dev/null; }

have_topic() { ros2 topic list 2>/dev/null | grep -qx -- "$1"; }

# wait until a topic exists AND delivers a message
wait_topic() {                # wait_topic /topic seconds "what it means"
  local topic="$1" limit="$2" meaning="${3:-}" t=0
  printf '    waiting for %s ' "$topic"
  while [ "$t" -lt "$limit" ]; do
    if have_topic "$topic" && timeout 3 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      printf ' %salive%s (%ss)\n' "$C_OK" "$C_0" "$t"; return 0
    fi
    printf '.'; sleep 2; t=$((t+2))
  done
  printf ' %stimeout%s\n' "$C_ERR" "$C_0"
  [ -n "$meaning" ] && say "      $meaning"
  return 1
}

scan_topic() { [ "$DO_LANE" = 1 ] && echo "/scan_fused" || echo "/scan"; }

# ------------------------------------------------------------------- actions
cmd_down() {
  step "Stopping everything"
  for f in "$PIDDIR"/*.pid; do
    [ -e "$f" ] || continue
    local n p; n="$(basename "$f" .pid)"; p="$(cat "$f")"
    kill -- -"$p" 2>/dev/null && ok "stopped $n" || true
    rm -f "$f"
  done
  # anything that outlived its launcher
  for pat in 'gz sim' 'ign gazebo' 'ruby .*gz' parameter_bridge robot_state_publisher \
             rviz2 slam_toolbox async_slam_toolbox_node controller_server planner_server \
             bt_navigator behavior_server smoother_server velocity_smoother waypoint_follower \
             collision_monitor lifecycle_manager map_server amcl ekf_node \
             lane_detection_esda.py lane_detection.py lane_detection_FCN.py \
             lane_detection_twinlite.py waypoint_navigator.py; do
    pkill -9 -f "$pat" 2>/dev/null
  done
  sleep 1
  ok "done"
}

cmd_status() {
  setup_env
  step "Processes"
  local any=0
  for f in "$PIDDIR"/*.pid; do
    [ -e "$f" ] || continue
    local n; n="$(basename "$f" .pid)"; any=1
    if alive "$n"; then ok "$n running (pid $(cat "$f"))"
    else warn "$n NOT running -- see $LOGDIR/$n.log"; fi
  done
  [ "$any" = 0 ] && say "    nothing started by this script"

  step "Topics"
  for t in /clock /scan /scan_fused /map /odometry/filtered /cmd_vel /cmd_vel_nav \
           /lane_markers /keepout_filter_mask /goal_pose /plan; do
    if have_topic "$t"; then
      if timeout 8 ros2 topic echo --once --qos-reliability best_effort "$t" >/dev/null 2>&1; then
        ok "$t  publishing"
      elif timeout 8 ros2 topic echo --once --qos-reliability best_effort \
             --qos-durability transient_local "$t" >/dev/null 2>&1; then
        ok "$t  publishing (latched)"
      else
        warn "$t  exists but no message in 8s"
      fi
    else
      say "    --   $t  absent"
    fi
  done

  step "Clock sanity"
  local a b
  a="$(date -u +%s)"
  b="$(powershell.exe -NoProfile -Command '[DateTimeOffset]::UtcNow.ToUnixTimeSeconds()' 2>/dev/null | tr -d '\r')"
  if [ -n "$b" ]; then
    local d=$(( a > b ? a - b : b - a ))
    if [ "$d" -le 5 ]; then ok "WSL and Windows agree (${d}s apart)"
    else warn "WSL and Windows differ by ${d}s -- this breaks TF and Nav2. See SETUP_AND_RUN.md"; fi
  fi
}

cmd_goal() {
  setup_env
  local x="${1:?usage: ./demo.sh goal X Y}" y="${2:?usage: ./demo.sh goal X Y}"
  step "Sending goal ($x, $y) in map frame"
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {w: 1.0}}}}"
}

cmd_drive() {
  setup_env
  have_topic /lane_markers || die "/lane_markers is absent -- the lane detector is not running"
  ros2 node list 2>/dev/null | grep -q bt_navigator || die "Nav2 is not up -- run ./demo.sh up first"
  step "Autonomous driving (their navigator)"
  start DRIVE "ros2 run $PKG waypoint_navigator_recommendation.py --ros-args -p use_sim_time:=true"
  sleep 5
  alive DRIVE && ok "running -- ./demo.sh logs DRIVE to watch" \
               || die "navigator exited -- see $LOGDIR/DRIVE.log"
}

cmd_view() {
  setup_env
  step "Camera view"
  say "    The detector already opens its own window titled"
  say "      'Lane Detection - lane_detection_node'"
  say "    (left | right camera on top, white mask | Canny edges underneath)."
  say "    Alt-Tab to it -- it is probably behind Gazebo."
  say ""
  if command -v rqt_image_view >/dev/null 2>&1; then
    say "    Also opening the raw camera in rqt_image_view."
    start VIEW "rqt_image_view /camera/left/image_raw"
  else
    say "    rqt_image_view not installed (sudo apt install -y ros-humble-rqt-image-view)"
  fi
}

cmd_logs() { local n="${1:?usage: ./demo.sh logs SIM|LANE|SLAM|NAV|RVIZ}"; tail -f "$LOGDIR/$n.log"; }

cmd_up() {
  setup_env
  cmd_down >/dev/null 2>&1

  local spawn_x=0.0 spawn_y=0.0
  [ "$WORLD" = "igvc.sdf" ] && { spawn_x=11.0; spawn_y=0; }
  local world_path="$WS/install/$PKG/share/$PKG/worlds/$WORLD"
  [ -f "$world_path" ] || world_path="$WS/src/$PKG/worlds/$WORLD"
  [ -f "$world_path" ] || die "world not found: $WORLD"

  if [ "$DO_BUILD" = 1 ]; then
    step "Building"
    ( cd "$WS" && colcon build --symlink-install --packages-select "$PKG" ) \
      >"$LOGDIR/BUILD.log" 2>&1 || die "build failed -- see $LOGDIR/BUILD.log"
    ok "built"
    source "$WS/install/setup.bash"
  fi

  # 1 -------------------------------------------------------------------- sim
  step "1/5  Gazebo + robot   (world: $WORLD, spawn $spawn_x,$spawn_y)"
  start SIM "ros2 launch $PKG launch_sim.launch.py \
      use_lidar:=true world_file:=$world_path \
      spawn_x:=$spawn_x spawn_y:=$spawn_y \
      robot_model:=robot.urdf.xacro camera_mount:=front lidar_mount:=pole_top"
  wait_topic /clock 90 "Gazebo never published /clock. Check $LOGDIR/SIM.log" || die "simulation did not start"
  wait_topic /scan  60 "The LiDAR plugin did not come up." || warn "no /scan -- SLAM will have nothing to map"

  # 2 ------------------------------------------------------------ lane detect
  if [ "$DO_LANE" = 1 ]; then
    step "2/5  Lane detection   (method: $METHOD)"
    start LANE "ros2 run $PKG lane_detection_esda.py --ros-args \
        -p detector_method:=$METHOD -p use_sim_time:=true -p show_visualization:=true"
    wait_topic /scan_fused 60 \
      "NOTHING ELSE PUBLISHES /scan_fused. SLAM, Nav2 and RViz all subscribe to it,
      so without this the costmap stays empty and the robot will not move.
      Check $LOGDIR/LANE.log" || die "lane detection did not come up"
  else
    step "2/5  Lane detection SKIPPED -- stack will use /scan"
  fi

  # 3 ------------------------------------------------------------------- slam
  step "3/5  SLAM   (scan topic: $(scan_topic))"
  start SLAM "ros2 launch $PKG online_async_launch.py use_sim_time:=true scan_topic:=$(scan_topic)"
  wait_topic /map 90 "slam_toolbox produced no map. Check $LOGDIR/SLAM.log" || die "SLAM did not come up"

  # 4 ------------------------------------------------------------------- nav2
  if [ "$DO_NAV" = 1 ]; then
    step "4/5  Nav2"
    start NAV "ros2 launch $PKG navigation_launch.py use_sim_time:=true \
        odom_topic:=odometry/filtered map_subscribe_transient_local:=true \
        params_file:=$WS/install/$PKG/share/$PKG/config/nav2_params.yaml \
        scan_topic:=$(scan_topic)"
    local t=0
    printf '    waiting for Nav2 lifecycle '
    while [ "$t" -lt 90 ]; do
      if ros2 node list 2>/dev/null | grep -q bt_navigator && have_topic /plan; then
        printf ' %sactive%s (%ss)\n' "$C_OK" "$C_0" "$t"; break
      fi
      printf '.'; sleep 3; t=$((t+3))
    done
    [ "$t" -ge 90 ] && warn "Nav2 slow to activate -- check $LOGDIR/NAV.log"
  else
    step "4/5  Nav2 SKIPPED"
  fi

  # 5 ------------------------------------------------------------------- rviz
  if [ "$DO_RVIZ" = 1 ]; then
    step "5/5  RViz"
    start RVIZ "rviz2 -d $WS/install/$PKG/share/$PKG/config/view_bot.rviz --ros-args -p use_sim_time:=true"
  else
    step "5/5  RViz SKIPPED"
  fi

  # 6 ------------------------------------------------------------ autonomy
  if [ "$DO_DRIVE" = 1 ]; then
    if [ "$DO_NAV" != 1 ]; then
      warn "--drive needs Nav2; ignoring --no-nav would be wrong, so skipping autonomy"
    else
      step "6/6  Autonomous driving   (their waypoint_navigator_recommendation.py)"
      start DRIVE "ros2 run $PKG waypoint_navigator_recommendation.py --ros-args -p use_sim_time:=true"
      sleep 5
      if ! alive DRIVE; then
        warn "the navigator exited -- check $LOGDIR/DRIVE.log"
        warn "if it says 'No module named nav2_simple_commander':"
        warn "    sudo apt install -y ros-humble-nav2-simple-commander"
      else
        ok "navigator running -- it drives from /lane_markers, no goal needed"
      fi
    fi
  fi

  step "Up."
  say "    ./demo.sh status        check topics"
  say "    ./demo.sh goal 20 0     send a navigation goal"
  say "    ./demo.sh logs SIM      follow a log"
  say "    ./demo.sh down          stop everything"
  say ""
}

# ---------------------------------------------------------------------- main
ACTION="${1:-up}"; shift || true
while [ $# -gt 0 ]; do
  case "$1" in
    --world)   WORLD="$2"; shift 2 ;;
    --method)  METHOD="$2"; shift 2 ;;
    --no-lane) DO_LANE=0; shift ;;
    --no-nav)  DO_NAV=0;  shift ;;
    --no-rviz) DO_RVIZ=0; shift ;;
    --no-build) DO_BUILD=0; shift ;;
    --drive)   DO_DRIVE=1; shift ;;
    *) break ;;
  esac
done

case "${ACTION:-up}" in
  up)     cmd_up ;;
  down)   cmd_down ;;
  status) cmd_status ;;
  goal)   cmd_goal "$@" ;;
  drive)  cmd_drive ;;
  view)   cmd_view ;;
  logs)   cmd_logs "$@" ;;
  *) say "usage: ./demo.sh {up|down|status|drive|view|goal X Y|logs NAME} [--world W] [--method M] [--drive] [--no-lane] [--no-nav] [--no-rviz] [--no-build]"; exit 1 ;;
esac
