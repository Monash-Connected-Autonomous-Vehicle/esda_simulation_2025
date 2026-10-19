#!/usr/bin/env bash
# =====================================================================
#  ESDA SIM LAUNCHER (WSL) — interactive menu
#
#      ./run.sh
#
#  Same menu numbers you are used to. What changed underneath:
#
#   * All the launch/teardown logic now lives in esda_lib.sh, shared with
#     demo.sh, so the two cannot drift apart.
#
#   * STOP ALL is pid-based. The old version ran `pkill -9 -f` over a list
#     of BARE WORDS including `nav2`, `create`, `ruby`, `amcl` and `spawner`.
#     pkill -f matches any process whose whole command line contains the
#     string, so `-f nav2` would kill an editor with nav2_params.yaml open
#     and `-f create` matches almost anything. Each stage is now launched
#     into its own process group and only those groups are killed.
#
#   * Paths come from ESDA_SRC (this folder) instead of the old hard-coded
#     /mnt/c/uni/2026_sem1/... which no longer exists.
#
#   * New: g) toggles GPU rendering. Software GL (llvmpipe) rasterises every
#     camera and lidar frame on the CPU and is the single biggest CPU cost
#     in this project.
#
#  Prefer one command and no menu?   ./demo.sh up [--gpu] [--vision]
# =====================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB=""
for c in "$HERE/esda_lib.sh" "${ESDA_SRC:-}/esda_lib.sh" \
         "/mnt/c/work/felix/mcav/for_server/esda_lib.sh"; do
  [ -n "$c" ] && [ -f "$c" ] && { LIB="$c"; break; }
done
if [ -z "$LIB" ]; then
  echo "Cannot find esda_lib.sh."
  echo "Run run.sh from the for_server folder, or set ESDA_SRC to it:"
  echo "    ESDA_SRC=/mnt/c/work/felix/mcav/for_server ./run.sh"
  exit 1
fi
# shellcheck source=esda_lib.sh
source "$LIB"

VISION_MODE=""     # "", lane_cone, lane_only, full

pause() { read -rp "press enter..." _; }

gl_label() {
  if [ "$GPU_RENDER" = "1" ]; then echo "GPU (hardware OpenGL)"; else echo "software (llvmpipe)"; fi
}

menu() {
  clear
  echo "==================== ESDA SIM LAUNCHER (WSL) ===================="
  echo "  rendering: $(gl_label)      source: $ESDA_SRC"
  echo "  ---------------------------------------------------------------"
  echo "  1) Launch SIM          (headless Gazebo + controllers + twist_mux + RViz)"
  echo "  2) SLAM  (mapping)     — add /map in RViz, drive to build the map"
  echo "  3) Nav2  (autonomous)  — then click '2D Goal Pose' in RViz"
  echo "  4) Teleop drive        (runs HERE — Ctrl+C returns to the menu)"
  echo "  5) RViz only"
  echo "  6) Capture camera frames (for a YOLO dataset)"
  echo "  8) Lane/cone vision node (YOLOv8-seg -> /lane/debug_image)"
  echo "  9) Generate SYNTHETIC labelled dataset (no Gazebo, no labelling)"
  echo " 10) Train YOLOv8-seg on the dataset (GPU)"
  echo "  ---------------------------------------------------------------"
  echo "  a) ALL-IN-ONE: sim + SLAM + Nav2   (what demo.sh up does)"
  echo "  n) Send a nav goal"
  echo "  g) Toggle GPU rendering   (currently: $(gl_label))"
  echo "  c) gpu_check.sh — what can run on the GPU"
  echo "  7) Status (what is running)"
  echo "  0) STOP ALL"
  echo "  q) Quit (also stops all)"
  echo "================================================================"
}

trap 'echo; echo "Ctrl+C — back to the menu"; sleep 1' INT

while true; do
  menu
  read -rp "choose: " c
  case "$c" in
    1) start_sim
       wait_for "/clock" 90 has_topic /clock || warn "see $LOGS/sim.log"
       pause ;;
    2) start_slam
       wait_for "/map" 90 has_pub /map || warn "see $LOGS/slam.log"
       pause ;;
    3) start_nav2 "$( [ -n "$VISION_MODE" ] && echo vision )"
       wait_for "bt_navigator active" 120 node_active /bt_navigator \
         || warn "see $LOGS/nav2.log"
       pause ;;
    4) src
       echo ">> teleop on /cmd_vel_joy (twist_mux priority 100, overrides Nav2)"
       echo ">> i/j/k/l to drive, Ctrl+C to return"
       ros2 run teleop_twist_keyboard teleop_twist_keyboard \
         --ros-args -r /cmd_vel:=/cmd_vel_joy ;;
    5) start_rviz; sleep 1 ;;
    6) src
       read -rp "interval sec [1.0]: " iv; iv="${iv:-1.0}"
       echo ">> capturing to ~/lane_dataset  (Ctrl+C to stop)"
       python3 "$ESDA_SRC/capture_frames.py" /head_camera/images "$iv" "$HOME/lane_dataset" ;;
    8) read -rp "mode [full/lane_cone/lane_only, enter=config]: " vm
       VISION_MODE="$vm"
       start_vision "${vm:+--mode $vm}"
       wait_for "/lane/debug_image" 60 has_pub /lane/debug_image \
         || warn "see $LOGS/vision.log"
       pause ;;
    9) src
       read -rp "train images [1600]: " nt; nt="${nt:-1600}"
       python3 "$ESDA_SRC/vision/synth_gen.py" --out "$HOME/esda_datasets/lane_seg" \
               --n-train "$nt" --n-val 280 --preview 8
       pause ;;
    10) src
       python3 "$ESDA_SRC/vision/train.py" \
               --data "$HOME/esda_datasets/lane_seg/data.yaml" \
               --model yolov8s-seg.pt --epochs 120 --imgsz 640 --device 0
       pause ;;
    a|A) # array, not an unquoted substitution: an empty string would reach
         # demo.sh as an argument and be rejected as an unknown flag
         up_args=()
         [ "$GPU_RENDER" = "1" ] && up_args+=(--gpu)
         [ -n "$VISION_MODE" ] && up_args+=(--vision)
         "$ESDA_SRC/demo.sh" up "${up_args[@]}"; pause ;;
    n|N) read -rp "goal x [2.0]: " gx; read -rp "goal y [0.0]: " gy
       "$ESDA_SRC/demo.sh" goal "${gx:-2.0}" "${gy:-0.0}"; pause ;;
    g|G) if [ "$GPU_RENDER" = "1" ]; then
           GPU_RENDER=0
           echo ">> rendering set to SOFTWARE"
         else
           GPU_RENDER=1
           echo ">> rendering set to GPU."
           echo "   ogre2 on the WSL D3D12 layer can kill the Gazebo sensors"
           echo "   mid-run. If /scan dies, toggle back and relaunch."
         fi
         echo ">> applies to stages started from now on; restart them to take effect"
         pause ;;
    c|C) bash "$ESDA_SRC/gpu_check.sh"; pause ;;
    7) stack_status; pause ;;
    0) stop_all; pause ;;
    q|Q) stop_all; exit 0 ;;
    *) ;;
  esac
done
