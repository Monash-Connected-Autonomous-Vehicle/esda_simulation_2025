#!/usr/bin/env bash
# =====================================================================
#  ESDA — what can actually run on the GPU in this WSL box?
#
#      bash gpu_check.sh
#      bash gpu_check.sh --weights ~/esda_models/lane_seg.pt
#
#  Checks four separate things that people tend to conflate:
#
#    1. Does WSL see the GPU at all?          (/dev/dxg, nvidia-smi)
#    2. Can OpenGL use it?                    (glxinfo renderer string)
#         -> this is what Gazebo and RViz need. It is ALSO the biggest CPU
#            cost in this project right now: LIBGL_ALWAYS_SOFTWARE=1 forces
#            llvmpipe, so every camera and lidar frame is rendered on the CPU.
#    3. Can PyTorch use it?                   (torch.cuda)
#         -> training, and the YOLO node.
#    4. How much faster is YOLO actually?     (measured, cpu vs cuda vs fp16)
#
#  Nothing here changes any settings. It only reports, then tells you which
#  switches are worth flipping.
# =====================================================================
set -uo pipefail

WEIGHTS="${HOME}/esda_models/lane_seg.pt"
while [ $# -gt 0 ]; do
  case "$1" in
    --weights) WEIGHTS="$2"; shift 2 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 1 ;;
  esac
done

hdr()  { echo; echo -e "\033[1;36m=== $* ===\033[0m"; }
ok()   { echo -e "  \033[1;32mOK\033[0m    $*"; }
bad()  { echo -e "  \033[1;31mNO\033[0m    $*"; }
info() { echo -e "        $*"; }

GL_HW=0; CUDA_OK=0

# ------------------------------------------------------------------ 1 ------
hdr "1. Does WSL see the GPU"
if grep -qi microsoft /proc/version 2>/dev/null; then
  ok "running under WSL"
else
  info "not WSL (that is fine, the rest still applies)"
fi

if [ -e /dev/dxg ]; then
  ok "/dev/dxg present  (WSL GPU passthrough is active)"
else
  bad "/dev/dxg missing -- WSL cannot see any GPU."
  info "Update the NVIDIA driver on WINDOWS (not inside WSL), then 'wsl --shutdown'."
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  if nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null; then
    ok "nvidia-smi works"
  else
    bad "nvidia-smi present but returned nothing"
  fi
else
  bad "no nvidia-smi inside WSL"
fi

if [ -d /usr/lib/wsl/lib ]; then
  ok "/usr/lib/wsl/lib present ($(find /usr/lib/wsl/lib -maxdepth 1 -type f 2>/dev/null | wc -l) libs)"
else
  bad "/usr/lib/wsl/lib missing -- GPU GL will not work"
fi

# ------------------------------------------------------------------ 2 ------
hdr "2. OpenGL — this is what Gazebo and RViz use"
if ! command -v glxinfo >/dev/null 2>&1; then
  bad "glxinfo not installed.  sudo apt install -y mesa-utils   then re-run"
else
  soft_r="$(LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
  hw_r="$(LIBGL_ALWAYS_SOFTWARE=0 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
          glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')"
  info "with LIBGL_ALWAYS_SOFTWARE=1 : ${soft_r:-<failed>}"
  info "with LIBGL_ALWAYS_SOFTWARE=0 : ${hw_r:-<failed>}"
  if echo "$hw_r" | grep -qiE "llvmpipe|softpipe|swrast"; then
    bad "hardware path still lands on a software renderer"
    info "Gazebo will keep burning CPU. Usually a Windows-side driver update."
  elif [ -n "$hw_r" ]; then
    ok "hardware OpenGL available: $hw_r"
    GL_HW=1
    ver="$(LIBGL_ALWAYS_SOFTWARE=0 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL core profile version string: //p')"
    info "GL core version: ${ver:-unknown}   (ogre2 needs 3.3+)"
  else
    bad "glxinfo produced nothing on the hardware path"
  fi
fi

# ------------------------------------------------------------------ 3 ------
hdr "3. PyTorch"
if python3 -c "import torch" 2>/dev/null; then
  python3 - <<'PY'
import torch
print(f"        torch {torch.__version__}  built for CUDA {torch.version.cuda}")
if torch.cuda.is_available():
    print(f"  \033[1;32mOK\033[0m    CUDA available: {torch.cuda.get_device_name(0)}")
    import time
    a = torch.randn(2000, 2000)
    t0 = time.time()
    for _ in range(8):
        a @ a
    cpu = (time.time() - t0) / 8
    g = a.cuda(); torch.cuda.synchronize()
    t0 = time.time()
    for _ in range(8):
        g @ g
    torch.cuda.synchronize()
    gpu = (time.time() - t0) / 8
    print(f"        2000x2000 matmul:  cpu {cpu*1000:6.1f} ms   gpu {gpu*1000:6.1f} ms"
          f"   ({cpu/max(gpu,1e-9):.0f}x)")
    raise SystemExit(0)
else:
    print("  \033[1;31mNO\033[0m    torch cannot see CUDA")
    if torch.version.cuda is None:
        print("        This is a CPU-ONLY torch build. Install a CUDA wheel:")
        print("          pip install --force-reinstall torch torchvision \\")
        print("            --index-url https://download.pytorch.org/whl/cu121")
    else:
        print("        CUDA-enabled build, but no device. Check the Windows driver.")
    raise SystemExit(1)
PY
  [ $? -eq 0 ] && CUDA_OK=1
else
  bad "torch not installed (bash setup_wsl.sh installs it)"
fi

# ------------------------------------------------------------------ 4 ------
hdr "4. YOLO inference — the number that decides the vision node"
if [ "$CUDA_OK" != "1" ]; then
  info "skipped (no CUDA)"
elif [ ! -f "$WEIGHTS" ]; then
  info "skipped: no weights at $WEIGHTS"
  info "pass --weights /path/to/lane_seg.pt"
else
  python3 - "$WEIGHTS" <<'PY'
import sys, time, numpy as np
try:
    from ultralytics import YOLO
except ImportError:
    print("        ultralytics not installed"); raise SystemExit
w = sys.argv[1]
img = (np.random.rand(480, 640, 3) * 255).astype('uint8')

def bench(dev, half, n=12):
    m = YOLO(w)
    kw = dict(imgsz=640, conf=0.35, device=dev, verbose=False)
    if half:
        kw["half"] = True
    for _ in range(3):
        m.predict(img, **kw)          # warm-up
    t0 = time.time()
    for _ in range(n):
        m.predict(img, **kw)
    return (time.time() - t0) / n * 1000

try:
    c = bench("cpu", False)
    print(f"        cpu          {c:7.1f} ms/frame   ({1000/c:5.1f} fps)")
except Exception as e:
    c = None; print("        cpu benchmark failed:", e)
for half in (False, True):
    try:
        g = bench("0", half)
        tag = "cuda fp16" if half else "cuda fp32"
        sp = f"   {c/g:4.1f}x vs cpu" if c else ""
        print(f"        {tag:<12} {g:7.1f} ms/frame   ({1000/g:5.1f} fps){sp}")
    except Exception as e:
        print(f"        cuda {'fp16' if half else 'fp32'} failed:", e)
print()
print("        The node only needs 5 fps (the sim camera rate), so even cpu may")
print("        keep up -- but every ms there is a ms Gazebo does not get.")
PY
fi

# ------------------------------------------------------------------ v ------
hdr "VERDICT — what to turn on"
if [ "$GL_HW" = "1" ]; then
  echo "  * Gazebo/RViz on the GPU:   ./demo.sh up --gpu"
  echo "    Biggest win available. Software GL renders every camera and lidar"
  echo "    frame on the CPU; this hands that to the GPU."
  echo "    If Gazebo crashes or the sensors die, drop back to ./demo.sh up."
else
  echo "  * Gazebo/RViz must stay on software GL for now (see section 2)."
fi
if [ "$CUDA_OK" = "1" ]; then
  echo "  * YOLO on the GPU:          already configured"
  echo "      vision_config.yaml has  model.device: \"0\"  -- it will use CUDA."
  echo "    For roughly 2x more, enable FP16:"
  echo "      vision_config.yaml ->  model.half: true"
  echo "      or test it first:      python3 vision/lane_seg_node.py --device 0 --half"
  echo "  * Training on the GPU:      python3 vision/train.py --device 0"
  echo "  * Fastest inference:        export a TensorRT engine ON THIS MACHINE"
  echo "      python3 vision/export_jetson.py ~/esda_models/lane_seg.pt \\"
  echo "              --format engine --half"
  echo "      then point vision_config.yaml model.weights at the .engine"
else
  echo "  * YOLO and training will run on CPU until torch can see CUDA."
fi
echo
echo "  Not a GPU switch, but the other big CPU lever is the physics step."
echo "  worlds/igvc.sdf uses max_step_size 0.001 (1000 steps/sec). Raising it"
echo "  to 0.002 roughly halves physics CPU at some fidelity cost."
echo
