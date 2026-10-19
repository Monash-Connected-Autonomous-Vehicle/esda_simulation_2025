#!/usr/bin/env bash
# =====================================================================
#  ESDA GPU-SERVER BOOTSTRAP  —  zero to "sim + training ready"
#  Target: WSL2 Ubuntu 22.04 (jammy) with an NVIDIA GPU.
#
#  Run INSIDE WSL Ubuntu 22.04:
#      bash setup_server.sh            # full install
#      bash setup_server.sh --no-build # skip the colcon build (just deps)
#
#  Idempotent-ish: safe to re-run. Installs ROS2 Humble, Gazebo Fortress,
#  Nav2/SLAM/ros2_control, the sim workspace, and GPU PyTorch + ultralytics.
#  Prints a VERIFY block at the end so the server-side Claude can confirm.
# =====================================================================
set -e

# ---------------------------------------------------------------------
#  SUPERSEDED BY ../setup_wsl.sh  — read this before using it.
#
#  This script git-clones the upstream repo into the workspace and then
#  copies the local folder over the top. That was fine when upstream was the
#  source of truth. It no longer is: the local folder now carries fixes that
#  do not exist upstream, among them
#
#    * nav2_params.yaml with the KEEPOUT_ZONE_ENABLED / SPEED_ZONE_ENABLED
#      placeholders resolved (unfixed, Nav2's costmaps refuse to configure);
#    * launch_sim_headless.launch.py with the twist_mux node that joins
#      Nav2's /cmd_vel to the drive controller (without it the robot never
#      moves);
#    * the whole vision/ pipeline and the lane linker.
#
#  A clone that half-succeeds, or an upstream layout change, can leave those
#  behind. setup_wsl.sh does not clone at all: the local folder IS the source.
#  It also installs twist_mux and mesa-utils, checks that torch really has
#  CUDA, and has a fast --sync path.
#
#  So by default this forwards. Pass --force-legacy if you genuinely want the
#  old clone-based behaviour.
# ---------------------------------------------------------------------
if [ "${1:-}" != "--force-legacy" ]; then
  HERE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  NEW="$HERE_DIR/../setup_wsl.sh"
  echo "setup_server.sh is superseded by setup_wsl.sh (see the comment at the top)."
  if [ -f "$NEW" ]; then
    echo "Forwarding to: $NEW"
    echo "(run 'bash setup_server.sh --force-legacy' to use this script instead)"
    if [ $# -gt 0 ]; then
      # legacy flags (--no-build) are not understood by setup_wsl.sh, and it
      # exits on an unknown option, so drop them rather than hand over a
      # confusing failure. Its equivalents: --sync (re-copy + rebuild only),
      # --no-ml (skip torch/ultralytics).
      echo "NOTE: ignoring legacy argument(s): $*"
      echo "      setup_wsl.sh takes --sync and --no-ml."
    fi
    echo
    exec bash "$NEW"
  fi
  echo "Could not find $NEW -- continuing with the legacy path."
  echo
fi


# ---- EDIT THESE if your repos/paths differ -------------------------
SIM_REPO="${SIM_REPO:-https://github.com/Monash-Connected-Autonomous-Vehicle/esda_simulation_2025.git}"
WS="${WS:-$HOME/esda_ws}"
# where this vision/ folder lives (so we can symlink configs/models dirs)
VISION_DIR="${VISION_DIR:-$(cd "$(dirname "$0")" && pwd)}"
# --------------------------------------------------------------------

DO_BUILD=1
[ "$1" = "--no-build" ] && DO_BUILD=0

say() { echo -e "\n\033[1;36m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m!!  $*\033[0m"; }

# ---- 0. sanity: Ubuntu 22.04 + GPU ---------------------------------
say "Checking OS + GPU"
. /etc/os-release
if [ "$VERSION_ID" != "22.04" ]; then
  warn "This is Ubuntu $VERSION_ID. ROS2 Humble needs 22.04 (jammy)."
  warn "In Windows PowerShell:  wsl --install -d Ubuntu-22.04   then re-run this."
  exit 1
fi
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader || true
else
  warn "nvidia-smi not found inside WSL. Install the NVIDIA Windows driver with WSL/CUDA support."
  warn "Training will fall back to CPU (slow). Continuing anyway."
fi

# ---- 1. base tools -------------------------------------------------
say "Installing base tools"
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release \
     git python3-pip python3-venv build-essential

# ---- 2. ROS2 Humble ------------------------------------------------
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

if [ ! -f /opt/ros/humble/setup.bash ]; then
  say "Adding ROS2 apt repo + installing Humble (desktop)"
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
       | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt update
  sudo apt install -y ros-humble-desktop ros-dev-tools
else
  say "ROS2 Humble already present"
fi

# ---- 3. sim + nav stack packages -----------------------------------
say "Installing Gazebo Fortress + Nav2/SLAM/ros2_control + bridges"
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-twist-mux ros-humble-teleop-twist-keyboard \
  ros-humble-cv-bridge python3-opencv \
  ros-humble-xacro python3-colcon-common-extensions

# ---- 4. workspace + sim repo ---------------------------------------
say "Setting up workspace at $WS"
mkdir -p "$WS/src"
if [ ! -d "$WS/src/esda_simulation_2025" ]; then
  git clone "$SIM_REPO" "$WS/src/esda_simulation_2025"
else
  ( cd "$WS/src/esda_simulation_2025" && { git pull --ff-only || warn "sim repo: skip pull (local changes)"; } )
fi

# make sure the vision/ edits (launch_sim_headless, ign control, camera,
# rsp.launch, view_bot.rviz) travel with the repo. If this vision/ folder is
# a sibling of the repo checkout, copy the patched files in.
PATCHED="$VISION_DIR/.."
if [ -d "$PATCHED/esda_simulation_2025" ]; then
  say "Syncing patched sim files from $PATCHED/esda_simulation_2025"
  cp -r "$PATCHED/esda_simulation_2025/." "$WS/src/esda_simulation_2025/" 2>/dev/null || true
fi

if [ "$DO_BUILD" = "1" ]; then
  say "Building the workspace (colcon)"
  ros_source /opt/ros/humble/setup.bash
  ( cd "$WS" && colcon build --symlink-install )
fi

# ---- 5. Python: GPU PyTorch + ultralytics --------------------------
say "Installing ultralytics + PyTorch (GPU)"
pip install --upgrade pip
pip install ultralytics
python3 - <<'PY'
import torch
print("torch", torch.__version__, "cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("gpu:", torch.cuda.get_device_name(0))
else:
    print("NO CUDA -> training will be slow. If you have a GPU, install a CUDA torch wheel:")
    print("  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121")
PY

# ---- 6. model/dataset dirs -----------------------------------------
mkdir -p "$HOME/esda_models" "$HOME/esda_datasets/lane_seg" "$HOME/esda_logs" "$HOME/lane_dataset"

# ---- 7. shell convenience ------------------------------------------
if ! grep -q "esda_ws/install/setup.bash" "$HOME/.bashrc"; then
  say "Adding ROS sourcing + software rendering to ~/.bashrc"
  {
    echo ""
    echo "# ---- ESDA ----"
    echo "source /opt/ros/humble/setup.bash"
    echo "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash"
    echo "export LIBGL_ALWAYS_SOFTWARE=1   # keeps Gazebo sensors from crashing in WSL"
  } >> "$HOME/.bashrc"
fi

# ---- VERIFY block --------------------------------------------------
cat <<EOF

\033[1;32m====================  SETUP DONE  ====================\033[0m
Verify (open a NEW terminal so ~/.bashrc is sourced):

  ros2 --version
  ros2 pkg list | grep esda_simulation_2025
  python3 -c "import torch; print('cuda', torch.cuda.is_available())"
  ros2 launch esda_simulation_2025 launch_sim_headless.launch.py   # should start

Workspace : $WS
Models    : ~/esda_models        (trained weights go here)
Datasets  : ~/esda_datasets/lane_seg
Vision    : $VISION_DIR  (train.py, lane_seg_node.py, vision_config.yaml)

Next: follow vision/CLAUDE_TASKS.md  (data collection -> train -> evaluate).
======================================================
EOF
