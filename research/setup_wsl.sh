#!/usr/bin/env bash
# =====================================================================
#  ESDA — WSL setup, from zero to "the robot drives in Gazebo"
#
#  Run INSIDE WSL Ubuntu 22.04:
#      cd /mnt/c/work/felix/mcav/for_server
#      bash setup_wsl.sh              # full install + build
#      bash setup_wsl.sh --sync       # just re-copy sources + rebuild (fast)
#      bash setup_wsl.sh --no-ml      # skip torch/ultralytics (sim only)
#
#  Differences from the older setup_server.sh, which you can still use:
#    * NO git clone. Your local folder is the source of truth now -- it holds
#      the fixed nav2_params.yaml, the twist_mux wiring and the vision code.
#      Cloning upstream and copying over it risked clobbering those.
#    * Copies sources into the LINUX filesystem (~/esda_ws). Building a colcon
#      workspace directly on /mnt/c goes through the 9p filesystem and is
#      brutally slow; a copy plus `--sync` is far quicker in practice.
#    * Installs twist_mux (required now -- Nav2 cannot reach the wheels
#      without it) and image_view for recording.
#
#  Idempotent: safe to re-run.
#
#  SLOW CONNECTION? Run  bash fetch_deps.sh  first. It downloads the debs and
#  wheels into caches without installing anything, is resumable, and then this
#  script installs from cache in a couple of minutes. fetch_deps.sh also
#  explains why you should NOT install the CUDA Toolkit or a Linux NVIDIA
#  driver inside WSL -- the Windows driver plus pip torch is the whole
#  requirement, and a Linux driver in WSL actively breaks GPU passthrough.
# =====================================================================
set -uo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$HOME/esda_ws}"
PKG="esda_simulation_2025"

DO_ML=1
SYNC_ONLY=0
for a in "$@"; do
  case "$a" in
    --sync)  SYNC_ONLY=1 ;;
    --no-ml) DO_ML=0 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $a"; exit 1 ;;
  esac
done

say()  { echo -e "\n\033[1;36m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m!!  $*\033[0m"; }
die()  { echo -e "\033[1;31mXX  $*\033[0m"; exit 1; }

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

# ---------------------------------------------------------------- sources ---
sync_sources() {
  say "Copying sources  $SRC_DIR/$PKG  ->  $WS/src/$PKG"
  [ -d "$SRC_DIR/$PKG" ] || die "cannot find $SRC_DIR/$PKG -- run this from the for_server folder"
  mkdir -p "$WS/src"
  rm -rf "$WS/src/$PKG"
  cp -r "$SRC_DIR/$PKG" "$WS/src/$PKG"
  # strip Windows CRLF from anything Python/YAML/launch, which otherwise
  # produces baffling "bad interpreter" and YAML parse errors
  find "$WS/src/$PKG" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.xacro' \
       -o -name '*.sdf' -o -name '*.xml' \) -print0 | xargs -0 -r sed -i 's/\r$//'
  echo "    $(find "$WS/src/$PKG" -type f | wc -l) files"
}

build_ws() {
  say "Building the workspace"
  ros_source /opt/ros/humble/setup.bash \
    || die "ROS 2 Humble not found -- run without --sync first"
  ( cd "$WS" && colcon build --symlink-install ) || die "colcon build failed (scroll up)"
}

if [ "$SYNC_ONLY" = "1" ]; then
  sync_sources
  build_ws
  say "Synced. Open a NEW terminal (or: source $WS/install/setup.bash)"
  exit 0
fi

# ---------------------------------------------------------------- checks ----
say "Checking the environment"
if ! grep -qi microsoft /proc/version 2>/dev/null; then
  warn "This does not look like WSL. Continuing anyway."
fi
# shellcheck disable=SC1091
. /etc/os-release
echo "    distro: $PRETTY_NAME"
if [ "${VERSION_ID:-}" != "22.04" ]; then
  warn "ROS 2 Humble needs Ubuntu 22.04 (jammy); this is ${VERSION_ID:-unknown}."
  warn "In Windows PowerShell:   wsl --install -d Ubuntu-22.04"
  warn "then run this script inside THAT distro."
  die  "stopping before installing the wrong ROS"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null \
    || warn "nvidia-smi present but returned nothing"
else
  warn "No nvidia-smi inside WSL. The sim runs fine on CPU; only TRAINING is slow."
  warn "For GPU training, install the NVIDIA Windows driver with WSL/CUDA support."
fi

# ---------------------------------------------------------------- apt -------
say "Base tools"
sudo apt-get update -y || die "apt update failed (no network?)"
sudo apt-get install -y software-properties-common curl gnupg lsb-release \
     git python3-pip python3-venv build-essential python3-tk || die "base tools failed"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  say "Installing ROS 2 Humble (this is the long one, ~10 min)"
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg || die "could not fetch the ROS key"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
       | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update -y
  sudo apt-get install -y ros-humble-desktop ros-dev-tools || die "ROS install failed"
else
  say "ROS 2 Humble already present"
fi

say "Gazebo Fortress + Nav2 + SLAM + ros2_control + twist_mux"
sudo apt-get install -y \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-twist-mux ros-humble-teleop-twist-keyboard \
  ros-humble-cv-bridge python3-opencv \
  ros-humble-image-view ros-humble-rqt-image-view \
  ros-humble-xacro python3-colcon-common-extensions \
  mesa-utils \
  || die "package install failed"
#   mesa-utils provides glxinfo, which is the only way to tell hardware
#   OpenGL apart from llvmpipe. Without it you cannot see whether Gazebo
#   is rendering on the GPU or quietly burning CPU.
#   twist_mux is NOT optional any more: Nav2 publishes /cmd_vel but
#   diff_drive_controller listens on /diff_drive_base_controller/cmd_vel_unstamped,
#   and twist_mux is what joins them. Without it the robot never moves.

# ---------------------------------------------------------------- build -----
sync_sources
build_ws

# ---------------------------------------------------------------- python ----
if [ "$DO_ML" = "1" ]; then
  say "Python: ultralytics + PyTorch"
  python3 -m pip install --upgrade pip
  python3 -m pip install ultralytics

  # `pip install ultralytics` usually pulls a CUDA torch on linux/x86_64, but
  # not always. A CPU-only wheel silently makes both training and the vision
  # node far slower -- exactly what we are trying to avoid -- and nothing
  # warns you. So check explicitly and repair it.
  TORCH_CUDA_BUILD="$(python3 -c 'import torch;print(torch.version.cuda or "none")' 2>/dev/null || echo none)"
  TORCH_SEES_GPU="$(python3 -c 'import torch;print(int(torch.cuda.is_available()))' 2>/dev/null || echo 0)"
  echo "    torch CUDA build: $TORCH_CUDA_BUILD   sees a GPU: $TORCH_SEES_GPU"

  if [ "$TORCH_CUDA_BUILD" = "none" ] && command -v nvidia-smi >/dev/null 2>&1; then
    warn "CPU-only torch on a machine that HAS a GPU -- installing a CUDA wheel"
    if ! python3 -m pip install --force-reinstall torch torchvision \
         --index-url https://download.pytorch.org/whl/cu128; then
      warn "CUDA wheel install failed; staying on CPU torch"
    fi
    TORCH_SEES_GPU="$(python3 -c 'import torch;print(int(torch.cuda.is_available()))' 2>/dev/null || echo 0)"
  fi

  if [ "$TORCH_SEES_GPU" = "1" ]; then
    python3 -c 'import torch;print("    GPU:", torch.cuda.get_device_name(0))'
  else
    warn "torch cannot see a GPU -- training and the vision node will use the CPU."
    warn "Diagnose with:  bash gpu_check.sh"
  fi
fi

mkdir -p "$HOME/esda_models" "$HOME/esda_datasets/lane_seg" "$HOME/esda_logs" \
         "$HOME/lane_dataset" "$HOME/esda_runs"

# pre-stage the trained weights if they are sitting in the repo
if [ -f "$SRC_DIR/models/lane_seg_synth_cpu.pt" ] && [ ! -f "$HOME/esda_models/lane_seg.pt" ]; then
  cp "$SRC_DIR/models/lane_seg_synth_cpu.pt" "$HOME/esda_models/lane_seg.pt"
  echo "    installed lane_seg.pt from models/lane_seg_synth_cpu.pt"
fi

# YOLO base weights for training. Shipped in the repo so the first training
# run needs no network; train.py resolves a bare "yolov8s-seg.pt" to here.
if [ -d "$SRC_DIR/models/base_weights" ]; then
  mkdir -p "$HOME/esda_models/base"
  cp -n "$SRC_DIR/models/base_weights"/*.pt "$HOME/esda_models/base/" 2>/dev/null
  echo "    base weights -> ~/esda_models/base ($(find "$HOME/esda_models/base" -name '*.pt' | wc -l) files)"
fi

# ---------------------------------------------------------------- shell -----
if ! grep -q "esda_ws/install/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
  say "Adding ROS sourcing to ~/.bashrc"
  {
    echo ""
    echo "# ---- ESDA ----"
    echo "source /opt/ros/humble/setup.bash"
    echo "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash"
    echo "export LIBGL_ALWAYS_SOFTWARE=1   # keeps Gazebo sensors alive under WSL"
    echo "export ESDA_SRC=$SRC_DIR"
  } >> "$HOME/.bashrc"
fi

# ---------------------------------------------------------------- verify ----
say "VERIFY"
ros_source /opt/ros/humble/setup.bash
ros_source "$WS/install/setup.bash"
fail=0
chk() { printf '    %-46s' "$1"; shift; if "$@" >/dev/null 2>&1; then echo "OK"; else echo "FAIL"; fail=1; fi; }
chk "ros2 available"            command -v ros2
chk "package $PKG found"        bash -c "ros2 pkg list | grep -qx $PKG"
chk "twist_mux installed"       bash -c "ros2 pkg list | grep -qx twist_mux"
chk "slam_toolbox installed"    bash -c "ros2 pkg list | grep -qx slam_toolbox"
chk "nav2_bringup installed"    bash -c "ros2 pkg list | grep -qx nav2_bringup"
chk "ros_gz_sim installed"      bash -c "ros2 pkg list | grep -qx ros_gz_sim"
chk "nav2_params.yaml parses"   python3 -c "import yaml,sys;yaml.safe_load(open('$WS/install/$PKG/share/$PKG/config/nav2_params.yaml'))"
chk "no unsubstituted placeholders" bash -c "! grep -q 'ZONE_ENABLED' $WS/install/$PKG/share/$PKG/config/nav2_params.yaml"

echo
if [ "$fail" = "0" ]; then
  echo -e "\033[1;32m====================  SETUP OK  ====================\033[0m"
else
  echo -e "\033[1;31m============  SETUP FINISHED WITH FAILURES  ========\033[0m"
fi
cat <<EOF

Open a NEW terminal (so ~/.bashrc is sourced), then:

    cd $SRC_DIR
    bash gpu_check.sh       # what can run on the GPU, and what it actually buys
    ./demo.sh up            # sim + SLAM + Nav2 + RViz  (software GL)
    ./demo.sh up --gpu      # ...render on the GPU instead (big CPU saving)
    ./demo.sh goal 3 0      # drive to x=3, y=0   (or click 2D Goal Pose in RViz)
    ./demo.sh down

Workspace : $WS
Logs      : ~/esda_logs
Recordings: ~/esda_runs

After editing anything in $SRC_DIR:
    bash setup_wsl.sh --sync
EOF
