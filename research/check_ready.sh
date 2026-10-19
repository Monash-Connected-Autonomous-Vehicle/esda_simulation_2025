#!/usr/bin/env bash
# =====================================================================
#  check_ready.sh — one-shot readiness report for the ESDA stack in WSL.
#
#      bash check_ready.sh
#
#  Everything it prints also goes to wsl_verify.log NEXT TO THIS SCRIPT,
#  which means the log lands in the Windows folder (C:\work\...\for_server)
#  and can be read from the other side without copy-pasting a terminal.
#
#  It changes nothing. Safe to run while something else is installing,
#  though the numbers will only be final once setup_wsl.sh has exited.
# =====================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$HERE/wsl_verify.log"
WS="$HOME/esda_ws"
PKG="esda_simulation_2025"
SHARE="$WS/install/$PKG/share/$PKG"

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

PASS=0; FAIL=0
line() { printf '  %-30s %s\n' "$1" "$2"; }
yes_()  { PASS=$((PASS+1)); line "$1" "OK    ${2:-}"; }
no_()   { FAIL=$((FAIL+1)); line "$1" "FAIL  ${2:-}"; }
chk()   { if [ -e "$2" ]; then yes_ "$1" "$2"; else no_ "$1" "missing: $2"; fi; }
hdr()   { echo; echo "--- $* ------------------------------------------"; }

main() {
echo "ESDA readiness report      $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "host: $(uname -srm)   user: $USER   distro: $(. /etc/os-release 2>/dev/null; echo "${PRETTY_NAME:-?}")"

hdr "ROS 2 + workspace"
chk "ROS Humble"        /opt/ros/humble/setup.bash
chk "workspace built"   "$WS/install/setup.bash"
chk "package installed" "$SHARE"
chk "headless launch"   "$SHARE/launch/launch_sim_headless.launch.py"
chk "rviz config"       "$SHARE/config/view_bot.rviz"
chk "twist_mux params"  "$SHARE/config/twist_mux.yaml"

n=$(find "$SHARE/worlds" -name 'igvc*.sdf' 2>/dev/null | wc -l)
[ "$n" -ge 11 ] && yes_ "arenas installed" "$n worlds" || no_ "arenas installed" "$n (want 11: stock + 10)"
n=$(find "$SHARE/models/orange_igvc/materials/textures" -name 'ground*.png' 2>/dev/null | wc -l)
[ "$n" -ge 11 ] && yes_ "ground textures" "$n" || no_ "ground textures" "$n (want 11)"

# The placeholders were the reason the costmap could never configure: a bare
# KEEPOUT_ZONE_ENABLED string reaches a parameter declared as bool.
if [ -f "$SHARE/config/nav2_params.yaml" ]; then
  n=$(grep -c 'ZONE_ENABLED' "$SHARE/config/nav2_params.yaml" 2>/dev/null || true)
  [ "$n" = "0" ] && yes_ "nav2 placeholders gone" || no_ "nav2 placeholders" "$n left -- rerun: bash setup_wsl.sh --sync"
else
  no_ "nav2_params.yaml" "not installed"
fi
chk "nav2 vision variant" "$SHARE/config/nav2_params_vision.yaml"

hdr "ROS packages we depend on"
ros_source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
ros_source "$WS/install/setup.bash"   >/dev/null 2>&1 || true
for p in ros_gz_sim ros_gz_bridge twist_mux slam_toolbox nav2_bringup controller_manager diff_drive_controller teleop_twist_keyboard cv_bridge image_view; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then yes_ "$p"; else no_ "$p" "not installed"; fi
done

hdr "weights"
chk "lane_seg.pt"      "$HOME/esda_models/lane_seg.pt"
n=$(find "$HOME/esda_models/base" -name '*.pt' 2>/dev/null | wc -l)
[ "$n" -ge 1 ] && yes_ "YOLO base weights" "$n" || no_ "YOLO base weights" "none in ~/esda_models/base"

hdr "python / GPU"
if python3 -c 'import ultralytics' 2>/dev/null; then
  yes_ "ultralytics" "$(python3 -c 'import ultralytics;print(ultralytics.__version__)' 2>/dev/null)"
else
  no_ "ultralytics" "pip install ultralytics"
fi
if command -v nvidia-smi >/dev/null 2>&1; then
  yes_ "nvidia-smi" "$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | head -1)"
else
  no_ "nvidia-smi" "no GPU passthrough"
fi
chk "/dev/dxg (WSL GPU)" /dev/dxg

echo
echo "  --- torch / Blackwell kernel test ---"
python3 - <<'PY'
import sys
try:
    import torch
except Exception as e:
    print("  torch                          FAIL  cannot import: %s" % e); sys.exit(0)
print("  torch                          OK    %s   cuda build: %s"
      % (torch.__version__, torch.version.cuda))
if not torch.cuda.is_available():
    print("  torch sees GPU                 FAIL  is_available() is False")
    print("  -> training and the vision node will run on the CPU. Diagnose: bash gpu_check.sh")
    sys.exit(0)
cap = torch.cuda.get_device_capability(0)
print("  torch sees GPU                 OK    %s   capability sm_%d%d"
      % (torch.cuda.get_device_name(0), cap[0], cap[1]))
# A CUDA wheel older than the GPU's architecture reports is_available()==True
# and then fails on the FIRST REAL KERNEL. So run one.
try:
    import time
    a = torch.randn(2048, 2048, device='cuda')
    b = torch.randn(2048, 2048, device='cuda')
    c = a @ b; torch.cuda.synchronize()
    torch.cuda.synchronize(); t0 = time.time()
    for _ in range(20):
        c = a @ b
    torch.cuda.synchronize()
    tf = 20 * 2 * 2048**3 / (time.time() - t0) / 1e12
    print("  KERNEL TEST                    PASS  matmul ran on the GPU, ~%.1f TFLOP/s fp32" % tf)
except Exception as e:
    print("  KERNEL TEST                    FAIL  %s" % str(e).split('\n')[0])
    if cap[0] >= 12:
        print("  -> sm_%d%d needs a cu128 wheel. Fix with:" % cap)
        print("     python3 -m pip install --force-reinstall torch torchvision \\")
        print("        --index-url https://download.pytorch.org/whl/cu128")
PY

hdr "OpenGL (does --gpu buy anything?)"
if command -v glxinfo >/dev/null 2>&1; then
  sw="$(LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
  hw="$(LIBGL_ALWAYS_SOFTWARE=0 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
        LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-} \
        glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
  line "software GL" "${sw:-unavailable}"
  if [ -z "$hw" ]; then
    line "hardware GL" "unavailable -- stay on software GL (default)"
  elif echo "$hw" | grep -qiE 'llvmpipe|softpipe|swrast'; then
    line "hardware GL" "$hw  <- still software, --gpu buys nothing"
  else
    line "hardware GL" "$hw  <- --gpu is worth trying"
  fi
else
  line "glxinfo" "not installed (sudo apt install -y mesa-utils)"
fi

hdr "verdict"
echo "  $PASS checks OK, $FAIL failed"
if [ "$FAIL" = "0" ]; then
  echo
  echo "  Ready. Next, with NO vision and NO gpu:"
  echo "      ./demo.sh up"
  echo "      ./demo.sh goal 3 0        # in a second terminal"
else
  echo
  echo "  Fix the FAILs above first. If the workspace or arenas are missing,"
  echo "  setup_wsl.sh has not finished -- let it run, then re-run this."
fi
echo
echo "  (this report was also written to $LOG)"
}

main 2>&1 | tee "$LOG"
