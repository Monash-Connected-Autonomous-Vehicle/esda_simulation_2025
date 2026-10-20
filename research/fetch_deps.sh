#!/usr/bin/env bash
# =====================================================================
#  ESDA — download everything, install nothing.
#
#      bash fetch_deps.sh            # fetch all of it
#      bash fetch_deps.sh --apt      # just the ROS/Gazebo debs
#      bash fetch_deps.sh --pip      # just the python wheels
#      bash fetch_deps.sh --check    # report only, download nothing
#
#  Separate from setup_wsl.sh on purpose: downloading is the slow, flaky,
#  bandwidth-hungry part, and it is resumable. Run this on a good connection,
#  then setup_wsl.sh installs from cache in a couple of minutes.
#
#  ---------------------------------------------------------------------
#  READ THIS BEFORE YOU INSTALL CUDA. You almost certainly must not.
#  ---------------------------------------------------------------------
#  On WSL2 the GPU works like this:
#
#    * The NVIDIA driver is installed on WINDOWS. Only on Windows.
#      WSL then exposes it inside Linux at /usr/lib/wsl/lib (libcuda.so,
#      libnvidia-*). That is why nvidia-smi works in WSL without you
#      installing anything there.
#
#    * DO NOT install a Linux NVIDIA driver inside WSL. The usual
#      `apt install nvidia-driver-xxx` or the .run installer overwrites
#      libcuda.so in the container and BREAKS the passthrough. This is the
#      single most common way people lose GPU access in WSL, and it is
#      annoying to undo.
#
#    * You do NOT need the CUDA Toolkit either. PyTorch's wheels ship their
#      own CUDA runtime -- installing torch pulls nvidia-cublas, nvidia-cudnn,
#      nvidia-cuda-runtime and friends as ordinary pip dependencies. The
#      Toolkit (nvcc) is only needed to COMPILE CUDA code, which nothing in
#      this project does.
#
#  So the CUDA story here is: update the Windows driver, then pip install
#  torch. That is the whole thing. This script checks the first and fetches
#  the second.
# =====================================================================
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CACHE="${ESDA_CACHE:-$HOME/esda_cache}"
WHEELS="$CACHE/wheels"
DO_APT=1; DO_PIP=1; DO_WEIGHTS=1; CHECK_ONLY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --apt)     DO_PIP=0; DO_WEIGHTS=0 ;;
    --pip)     DO_APT=0; DO_WEIGHTS=0 ;;
    --weights) DO_APT=0; DO_PIP=0 ;;
    --check)   CHECK_ONLY=1 ;;
    -h|--help) sed -n '2,42p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 1 ;;
  esac
  shift
done

say()  { echo; echo -e "\033[1;36m==> $*\033[0m"; }
ok()   { echo -e "  \033[1;32mOK\033[0m   $*"; }
bad()  { echo -e "  \033[1;31mNO\033[0m   $*"; }
info() { echo -e "       $*"; }

mkdir -p "$CACHE" "$WHEELS"

# ------------------------------------------------------------------ GPU ----
say "GPU preflight (nothing is installed here)"
if [ -e /dev/dxg ]; then ok "/dev/dxg present -- WSL GPU passthrough active"
else bad "/dev/dxg missing -- update the NVIDIA driver on WINDOWS, then 'wsl --shutdown'"; fi

if command -v nvidia-smi >/dev/null 2>&1; then
  if nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null; then
    ok "nvidia-smi works (the Windows driver is visible from WSL)"
  fi
else
  bad "no nvidia-smi in WSL -- the Windows driver is missing or too old"
fi

if [ -f /usr/lib/wsl/lib/libcuda.so.1 ]; then
  ok "/usr/lib/wsl/lib/libcuda.so.1 present (this is the one that matters)"
else
  bad "/usr/lib/wsl/lib/libcuda.so.1 missing"
fi

# the mistake this script exists to prevent
if dpkg -l 2>/dev/null | grep -qE '^ii\s+(nvidia-driver-[0-9]+|libnvidia-gl-[0-9]+)'; then
  bad "A LINUX NVIDIA DRIVER IS INSTALLED INSIDE WSL."
  info "That overwrites libcuda.so and breaks the Windows passthrough."
  info "Remove it:   sudo apt purge '^nvidia-.*' && sudo apt autoremove"
  info "then 'wsl --shutdown' from PowerShell and reopen."
else
  ok "no Linux NVIDIA driver inside WSL (correct)"
fi

if [ "$CHECK_ONLY" = "1" ]; then say "check only -- nothing downloaded"; exit 0; fi

# ------------------------------------------------------------------ apt ----
if [ "$DO_APT" = "1" ]; then
  say "Downloading ROS 2 Humble + Gazebo debs (not installing)"
  if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    info "adding the ROS apt repo first (needed even just to download)"
    sudo apt-get install -y -qq curl gnupg lsb-release software-properties-common >/dev/null
    sudo add-apt-repository -y universe >/dev/null
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg || bad "could not fetch the ROS key"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
  sudo apt-get update -y
  # --download-only fills /var/cache/apt/archives; setup_wsl.sh then installs
  # from cache without touching the network. Resumable: re-run after a drop.
  if sudo apt-get install -y --download-only \
    ros-humble-desktop ros-dev-tools \
    ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
    ros-humble-ign-ros2-control \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-twist-mux ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge python3-opencv \
    ros-humble-image-view ros-humble-rqt-image-view \
    ros-humble-xacro python3-colcon-common-extensions \
    mesa-utils python3-tk
  then
    ok "debs cached in /var/cache/apt/archives ($(du -sh /var/cache/apt/archives 2>/dev/null | cut -f1))"
  else
    bad "apt download failed -- re-run, it resumes"
  fi
fi

# ------------------------------------------------------------------ pip ----
if [ "$DO_PIP" = "1" ]; then
  say "Downloading python wheels into $WHEELS"
  info "torch's CUDA runtime arrives as pip deps (nvidia-cublas, nvidia-cudnn,"
  info "nvidia-cuda-runtime, ...). This is several GB. No CUDA Toolkit needed."
  if python3 -m pip download --dest "$WHEELS" ultralytics; then
    ok "ultralytics + torch wheels cached"
  else
    bad "pip download failed -- re-run, it skips what it already has"
  fi

  # TensorRT is optional: only for exporting a .engine for faster inference.
  if python3 -m pip download --dest "$WHEELS" tensorrt >/dev/null 2>&1; then
    ok "tensorrt cached (optional, for export_jetson.py --format engine)"
  else
    info "tensorrt not cached (optional; skip unless you want a TRT engine)"
  fi
  info "cache size: $(du -sh "$WHEELS" 2>/dev/null | cut -f1)"
  info "install later with:  pip install --no-index --find-links $WHEELS ultralytics"
fi

# -------------------------------------------------------------- weights ----
if [ "$DO_WEIGHTS" = "1" ]; then
  say "YOLO base weights (needed by train.py)"
  DEST="$HOME/esda_models/base"
  mkdir -p "$DEST"
  if [ -d "$HERE/models/base_weights" ]; then
    cp -n "$HERE/models/base_weights"/*.pt "$DEST"/ 2>/dev/null
    ok "copied pre-staged weights from the repo (no download needed)"
  fi
  for w in yolov8n-seg.pt yolov8s-seg.pt yolov8m-seg.pt; do
    if [ -s "$DEST/$w" ]; then
      info "$w already present ($(du -h "$DEST/$w" | cut -f1))"
    else
      printf '       fetching %s ... ' "$w"
      if curl -sL --max-time 300 -o "$DEST/$w" \
           "https://github.com/ultralytics/assets/releases/download/v8.3.0/$w"; then
        echo "$(du -h "$DEST/$w" | cut -f1)"
      else
        echo "FAILED"; rm -f "$DEST/$w"
      fi
    fi
  done
  ok "base weights in $DEST"
fi

say "Done"
cat <<EOF
  apt cache : /var/cache/apt/archives
  wheels    : $WHEELS
  weights   : $HOME/esda_models/base

  Next:   bash setup_wsl.sh       (installs from the caches above)
          bash gpu_check.sh       (confirms what actually runs on the GPU)

  Reminder: do NOT install a Linux NVIDIA driver or the CUDA Toolkit inside
  WSL. Windows driver + pip torch is the whole requirement.
EOF
