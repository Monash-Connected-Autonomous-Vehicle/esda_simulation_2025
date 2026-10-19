#!/usr/bin/env bash
# =====================================================================
#  esda_lib.sh — shared machinery for demo.sh, run.sh and anything else
#  that starts or stops the ESDA stack.  SOURCE this, do not execute it.
#
#      source "$(dirname "$0")/esda_lib.sh"
#
#  WHY THIS FILE EXISTS
#  run.sh, esda_gui.py and demo.sh each grew their own copy of "launch a
#  thing in the background" and "kill everything". Two of the three killed
#  processes with `pkill -9 -f <bare word>`, against a list that included
#  `nav2`, `create`, `ruby`, `amcl` and `spawner`. pkill -f matches any
#  process whose FULL COMMAND LINE contains the string, so:
#
#      pkill -9 -f nav2     kills your editor if it has nav2_params.yaml open
#      pkill -9 -f create   matches almost anything
#      pkill -9 -f ruby     kills unrelated ruby processes
#
#  This is not hypothetical: while testing demo.sh, `pkill -f bt_navigator`
#  killed the very shell running the test, because the word appeared in its
#  arguments.
#
#  So: every stage is launched with setsid into its OWN process group, the
#  pgid is recorded in $LOGS/<stage>.pid, and teardown kills exactly those
#  groups. The pattern fallback that remains uses full invocations
#  ("ros2 launch esda_simulation_2025"), never bare words.
# =====================================================================

# guard against double-sourcing
[ -n "${ESDA_LIB_LOADED:-}" ] && return 0
ESDA_LIB_LOADED=1

ESDA_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$ESDA_LIB_DIR"
ESDA_SRC="${ESDA_SRC:-$SCRIPT_DIR}"
WS="${WS:-$HOME/esda_ws}"
PKG="esda_simulation_2025"
LOGS="$HOME/esda_logs"
RUNS="$HOME/esda_runs"
mkdir -p "$LOGS" "$RUNS"

# Rendering backend. Default stays SOFTWARE because ogre2 on the WSL D3D12
# mapping layer has a history of killing the Gazebo sensors mid-run, and a
# dead /scan is worse than a busy CPU. --gpu opts in.
GPU_RENDER=0
render_env() {
  if [ "$GPU_RENDER" = "1" ]; then
    # shellcheck disable=SC2016
    #   single quotes are deliberate: ${LD_LIBRARY_PATH:-} must expand inside
    #   the child shell bg() spawns, not here, so the child keeps its own path
    printf '%s' 'export LIBGL_ALWAYS_SOFTWARE=0; export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA; export LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}; '
  else
    printf '%s' 'export LIBGL_ALWAYS_SOFTWARE=1; '
  fi
}

# ---------------------------------------------------------------------
#  ROS 2 setup files are NOT `set -u` safe.
#
#  /opt/ros/humble/setup.sh does      [ -n "$AMENT_TRACE_SETUP_FILES" ]
#  a colcon workspace's local_setup.bash does  [ -n "$COLCON_TRACE" ]
#
#  Both are unset in a normal shell. Under `set -u` that is an unbound
#  variable, and a NON-INTERACTIVE bash exits immediately -- silently, if
#  stderr was redirected. That is what killed this script right after the
#  apt phase: it printed "Building the workspace" and stopped, so ROS got
#  installed but the workspace was never built.
#
#  So: drop -u for the duration of the source, then restore it.
# ---------------------------------------------------------------------
ros_source() {
  local f="$1" had_u=0 rc=0
  case "$-" in *u*) had_u=1 ;; esac
  set +u
  if [ -f "$f" ]; then
    # shellcheck disable=SC1091
    . "$f" || rc=$?
  else
    rc=1
  fi
  [ "$had_u" = "1" ] && set -u
  return "$rc"
}

# Source both ROS environments. ALWAYS returns 0: callers use it as
# `src; ros2 topic list` and `if src && ros2 topic list ...`, so a missing
# workspace must not abort them.
src() {
  ros_source /opt/ros/humble/setup.bash  >/dev/null 2>&1 || true
  ros_source "$WS/install/setup.bash"    >/dev/null 2>&1 || true
  return 0
}
say()  { echo -e "\033[1;36m==> $*\033[0m"; }
ok()   { echo -e "    \033[1;32mOK\033[0m  $*"; }
warn() { echo -e "    \033[1;33m!!\033[0m  $*"; }
die()  { echo -e "\033[1;31mXX  $*\033[0m"; exit 1; }

# Launch a stage in its OWN process group (setsid), and remember the pid.
# Teardown then kills exactly what we started. The obvious alternative,
# `pkill -9 -f <word>`, kills anything whose command line merely CONTAINS the
# word -- during testing `pkill -f bt_navigator` killed the very shell running
# the test, because the word appeared in its arguments. Patterns like
# `-f create` or `-f ruby` are worse still.
bg() {
  local name="$1"; shift
  echo ">> $name   (log: $LOGS/$name.log)"
  setsid nohup bash -c "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; \
                 $(render_env) $*" >"$LOGS/$name.log" 2>&1 &
  echo $! > "$LOGS/$name.pid"
}

# kill a stage by the process group we created for it
kill_stage() {
  local name="$1" pidf="$LOGS/$1.pid" pid
  [ -f "$pidf" ] || return 0
  pid="$(cat "$pidf" 2>/dev/null)"
  [ -n "${pid:-}" ] || { rm -f "$pidf"; return 0; }
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.5
    done
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
    echo "    stopped $name (pgid $pid)"
  fi
  rm -f "$pidf"
}

# wait_for <description> <timeout-s> <command...>
# Two separate hazards, two separate bounds.
#
#   1. The BUDGET is wall time, not loop passes. The old version counted
#      iterations and assumed each cost about a second, so its 120 s meant
#      nothing once a check started taking longer.
#   2. Each individual CHECK is bounded too, because a budget cannot be
#      enforced from inside a call that never returns. Every predicate here
#      shells out to the ros2 CLI, which talks to the DDS graph; against a
#      node whose state service never answers, `ros2 lifecycle get` blocks
#      forever. That hung a benchmark run at "waiting for bt_navigator
#      active" for eighteen minutes without ever printing TIMEOUT.
#
# So the predicate runs in a subshell with a deadline, and what it spawned is
# cleaned up if it overruns. The predicates cap their own ros2 calls as well;
# this is the backstop for any that forgets.
wait_for() {
  local what="$1" t="$2"; shift 2
  local pt="${WAIT_PRED_TIMEOUT:-15}"
  printf '    waiting for %-34s' "$what"
  # A SUSPENDED MACHINE IS NOT A TIMEOUT.
  #
  # This measures elapsed time with the wall clock, and this laptop keeps
  # going to sleep. A bring-up that slept through an hour reported
  # "bt_navigator active TIMEOUT after 3607s" against a 120 s deadline --
  # 30x over, which no real hang can produce, because the loop itself cannot
  # run that long. Nav2 had not failed; the clock had jumped.
  #
  # So measure how long each PASS took. A pass is bounded by pt (the
  # predicate timeout) plus a second of slack; anything beyond that is the
  # machine having been asleep, and it is discounted rather than counted
  # against the deadline.
  local t0 el p k rc now last skew=0 jump
  t0=$(date +%s); last=$t0
  while :; do
    ( "$@" >/dev/null 2>&1 ) & p=$!
    k=0
    while kill -0 "$p" 2>/dev/null && [ "$k" -lt "$pt" ]; do sleep 1; k=$((k+1)); done
    if kill -0 "$p" 2>/dev/null; then
      pkill -P "$p" 2>/dev/null          # the ros2 call it is stuck inside
      kill -TERM "$p" 2>/dev/null
      wait "$p" 2>/dev/null
      rc=1
    else
      wait "$p"; rc=$?
    fi
    now=$(date +%s)
    jump=$(( now - last - pt - 2 ))
    if [ "$jump" -gt 0 ]; then
      skew=$(( skew + jump ))
      echo
      warn "the clock jumped ${jump}s while waiting for '$what' -- this
    machine suspended. Not counting it against the ${t}s deadline."
      printf '    waiting for %-34s' "$what"
    fi
    last=$now
    [ "$rc" = "0" ] && { echo "OK ($(( now - t0 - skew ))s)"; return 0; }
    el=$(( now - t0 - skew ))
    [ "$el" -ge "$t" ] && break
    [ "$k" -eq 0 ] && sleep 1
  done
  echo "TIMEOUT after $(( $(date +%s) - t0 - skew ))s"
  [ "$skew" -gt 0 ] && warn "(${skew}s of that was the machine asleep and was not counted)"
  return 1
}

has_topic()  { src; timeout "${ROS_CLI_TIMEOUT:-10}" ros2 topic list 2>/dev/null | grep -qx "$1"; }
# ros_gz_bridge advertises its ROS-side topics the instant the bridge node
# starts, with or without Gazebo behind them -- so has_topic/has_pub on /clock
# or /scan say "the bridge is alive", NOT "the simulation is running". When it
# matters, wait for a real message.
has_data()   { src; timeout "${2:-8}" ros2 topic echo --once "$1" >/dev/null 2>&1; }
has_pub()    { src; [ "$(timeout "${ROS_CLI_TIMEOUT:-10}" ros2 topic info "$1" 2>/dev/null | awk '/Publisher count/{print $3}')" -gt 0 ] 2>/dev/null; }
# A subscriber on the controller's command topic is the dependency-free way to
# know the controller is up and listening. `ros2 control list_controllers`
# would need ros2controlcli, which is a separate package.
has_sub()    { src; [ "$(timeout "${ROS_CLI_TIMEOUT:-10}" ros2 topic info "$1" 2>/dev/null | awk '/Subscription count/{print $3}')" -gt 0 ] 2>/dev/null; }
# Each of these shells out to the ros2 CLI, which talks to the DDS graph and
# can block indefinitely when a node is half-dead. Nothing here is worth more
# than a few seconds, so cap every one of them.
has_node()   { src; timeout "${ROS_CLI_TIMEOUT:-10}" ros2 node list 2>/dev/null | grep -q "$1"; }
node_active(){ src; [ "$(timeout "${ROS_CLI_TIMEOUT:-10}" ros2 lifecycle get "$1" 2>/dev/null | cut -d' ' -f1)" = "active" ]; }

# ---------------------------------------------------------------------
#  Stage launchers. One definition, used by demo.sh and run.sh alike.
# ---------------------------------------------------------------------

# Where the robot starts, for one world. Echoes "x y yaw", or nothing if the
# world is not listed -- an unlisted world spawns at the origin, which is right
# for the worlds that were built around it.
#
# The table is config/world_spawns.txt and it is the ONLY copy of this fact.
# It used to be a case block here and a 0,0,0 default in capture_labelled.py,
# which is how job 109 came to record 672 frames with every pose off by
# (11.86, -2.31, 1.263 rad). vision/test_spawn_table.py asserts the readers
# agree with the file.
WORLD_SPAWNS="${WORLD_SPAWNS:-$ESDA_SRC/esda_simulation_2025/config/world_spawns.txt}"
world_spawn() {
  [ -f "$WORLD_SPAWNS" ] || { warn "no spawn table at $WORLD_SPAWNS"; return 1; }
  awk -v w="$1" '{ sub(/#.*/, "") } $1 == w { print $2, $3, $4; exit }' "$WORLD_SPAWNS"
}

nav2_params_path() {
  # $1 = "vision" to get the vision-layer variant
  local f="nav2_params.yaml"
  [ "${1:-}" = "vision" ] && f="nav2_params_vision.yaml"
  # "show" is the vision params with both costmaps publishing again, for a
  # run somebody is watching or recording. The scoring runs switch that off
  # because nothing subscribes under --no-rviz.
  [ "${1:-}" = "show" ] && f="nav2_params_vision_show.yaml"
  echo "$WS/install/$PKG/share/$PKG/config/$f"
}

# $1 = world stem (igvc, igvc_oval, igvc_wavy, igvc_figure8, igvc_slalom,
# igvc_spline). Empty -> the stock igvc course.
start_sim()  {
  local w="${1:-}" sx="${2:-0.0}" sy="${3:-0.0}" syaw="${4:-0.0}" rv="${5:-true}"
  # ESDA_RENDER_ARGS goes to the Ignition server verbatim; empty means the
  # defaults, so an unset variable leaves behaviour exactly as it was.
  local ra=""
  [ -n "${ESDA_RENDER_ARGS:-}" ] && ra=" render_args:='${ESDA_RENDER_ARGS}'"
  bg sim "ros2 launch $PKG launch_sim_headless.launch.py${w:+ world:=$w} \
          spawn_x:=$sx spawn_y:=$sy spawn_yaw:=$syaw rviz:=$rv$ra"
}
start_slam() { bg slam "ros2 launch $PKG online_async_launch.py use_sim_time:=true"; }
start_nav2() {
  local p; p="$(nav2_params_path "${1:-}")"
  if [ ! -f "$p" ]; then
    warn "missing $p -- run: bash setup_wsl.sh --sync"
    return 1
  fi
  # No map_subscribe_transient_local here: navigation_launch.py never declares
  # that argument (its set is namespace/use_sim_time/autostart/params_file/
  # use_composition/container_name/use_respawn/log_level). The value already
  # lives in nav2_params.yaml under static_layer.
  bg nav2 "ros2 launch $PKG navigation_launch.py use_sim_time:=true params_file:=$p"
}
start_vision() {
  local extra="${1:-}"
  [ -f "$HOME/esda_models/lane_seg.pt" ] || \
    warn "no weights at ~/esda_models/lane_seg.pt -- node runs in pass-through mode"
  bg vision "python3 $ESDA_SRC/vision/lane_seg_node.py $extra"
}
# Attach a Gazebo GUI to the ALREADY RUNNING server. The sim is launched
# with -s (server only) because an ogre2 GUI inside the launch has a history
# of taking the sensors down with it when it dies. Running the GUI as its own
# process keeps that risk off the simulation: if the window crashes, physics,
# /scan and the controllers all carry on. stop_all's existing "ign gazebo"
# pattern already matches "ign gazebo -g", so teardown needs no change there.
start_gui() { bg gui "ign gazebo -g"; }

# The no-waypoint driver: reads /lane/points, aims a carrot down the lane
# centreline, republishes it as /goal_pose. Nav2 still owns obstacle
# avoidance -- this only decides WHERE to aim.
start_drive() { bg drive "python3 $ESDA_SRC/vision/auto_drive.py ${1:-}"; }

start_rviz() {
  bg rviz "rviz2 -d $WS/install/$PKG/share/$PKG/config/view_bot.rviz"
}

# Stop everything WE started, by process group. Safe: touches nothing else.
stop_all() {
  say "Stopping everything we started"
  local n
  for n in vid bag teleop drive vision nav2 slam rviz gui sim; do kill_stage "$n"; done

  # Fallback for things started by hand in another terminal. Every pattern is
  # a full invocation, never a bare word -- see the header for why.
  local p
  for p in \
      "ros2 launch $PKG" \
      "ros2 run teleop_twist_keyboard" \
      "vision/auto_drive.py" \
      "vision/lane_seg_node.py" \
      "capture_frames.py" \
      "ros2 bag record" \
      "image_view video_recorder" \
      "ign gazebo" \
      "rviz2 -d" \
      "ros_gz_bridge parameter_bridge" \
      "twist_mux --ros-args" \
      "async_slam_toolbox_node" ; do
    pkill -f -- "$p" 2>/dev/null
  done
  sleep 1
  echo "    done."
}

stack_status() {
  src
  say "Stages we launched"
  local found=0 n pidf pid
  for n in sim slam nav2 vision drive rviz gui bag vid; do
    pidf="$LOGS/$n.pid"
    [ -f "$pidf" ] || continue
    pid="$(cat "$pidf" 2>/dev/null)"
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      printf '    %-8s running (pgid %s)\n' "$n" "$pid"; found=1
    else
      printf '    %-8s not running (stale pidfile)\n' "$n"
    fi
  done
  [ "$found" = "0" ] && echo "    (nothing we launched is running)"

  say "Key topics"
  local t
  for t in /clock /scan /odom /map /cmd_vel /diff_drive_base_controller/cmd_vel_unstamped \
           /lane/points /lane/points_raw /cone/points /plan; do
    if ros2 topic list 2>/dev/null | grep -qx "$t"; then
      printf '    %-48s publishers: %s\n' "$t" \
        "$(ros2 topic info "$t" 2>/dev/null | awk '/Publisher count/{print $3}')"
    else
      printf '    %-48s %s\n' "$t" "absent"
    fi
  done

  say "Nav2 lifecycle"
  for t in /bt_navigator /planner_server /controller_server; do
    printf '    %-24s %s\n' "$t" "$(ros2 lifecycle get "$t" 2>/dev/null || echo '-')"
  done
}
