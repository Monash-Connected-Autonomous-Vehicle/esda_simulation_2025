# ESDA Vision — YOLOv8-seg lane + cone detection

> **Just want to run it?** See **[RUN_YOLO.md](RUN_YOLO.md)**. The trained
> weight is committed at `research/models/lane_seg_synth_cpu.pt`, so
> `python3 lane_seg_node.py --detector yolo` works from a fresh clone.
> This file is the design and the measurements behind it.

Replaces the classical white-threshold lane detector, which tagged
**white-striped barrels as lane** and inflated the Nav2 costmap into a false
wall the robot couldn't drive through. Here `lane` and `cone` are **separate
learned classes on separate topics**, so they feed different costmap layers and
can never be confused.

### Why a learned model, not a better threshold

Sampled from the sim's own textures:

| | colour (BGR) |
|---|---|
| lane paint (`ground.png`) | 242, 242, 242 |
| barrel reflective band (`Construction_Barrel_Diffuse.png`) | 235, 224, 216 |

Identical. Colour alone cannot separate them at any threshold; the model has to
learn shape and context. That is the entire justification for this pipeline.

## Where things run
- **GPU server (WSL2 Ubuntu 22.04)** — Gazebo sim, dataset generation,
  training, live inference. Set up with `setup_server.sh`.
- **Jetson (later)** — same `lane_seg_node.py`, weights exported to a TensorRT
  engine (`export_jetson.py`). Switch `vision_config.yaml` to the `.engine` and
  `half: true`.

## Files
| file | what |
|---|---|
| `setup_server.sh` | zero→ready: ROS2 Humble + Gazebo + sim build + GPU torch + ultralytics |
| `CLAUDE_TASKS.md` | step-by-step runbook (generate → train → deploy → Nav2) |
| **`synth_gen.py`** | **renders labelled frames from the sim's own textures. No Gazebo, no labelling, no Roboflow account** |
| `vision_config.yaml` | one config: model, topics, class scheme, ground projection |
| `data.yaml` | hand-layout dataset template (`synth_gen.py` writes its own) |
| `train.py` | train YOLOv8-seg on the GPU; copies `best.pt` → `~/esda_models/lane_seg.pt` |
| `train_cpu.py` | thin CPU-only wrapper used to validate the pipeline without a GPU |
| `lane_seg_node.py` | ROS2 inference: overlay + lane/cone pointclouds to the costmap |
| **`lane_link.py`** | **joins broken lane fragments back into continuous lane lines** |
| `test_lane_link.py` | verification for the linker (geometry cases + end-to-end) |
| `viz_link.py` | camera + bird's-eye figure showing which fragments were joined |
| `hard_case.py` | measures cone-pixels-called-lane on worst-case frames |
| `nav2_vision_costmap_snippet.yaml` | merge into `nav2_params.yaml` — the cone-as-lane fix |
| `export_jetson.py` | later: `.pt` → TensorRT/ONNX for the Jetson |

## Getting a dataset without labelling anything

```bash
python3 synth_gen.py --out ~/esda_datasets/lane_seg --n-train 1600 --n-val 280 --preview 8
```

It renders the camera view using the **same ground-plane homography**
`lane_seg_node.py` uses to project detections, built from the verified xacro
chain — so training and deployment share one geometry. Because the scene is
generated, every pixel's class is known exactly: labels are free and perfect,
and the barrels' white bands always belong to `cone`.

Randomised per frame: lane offset, heading, curvature, corridor width, paint
width, dashes, barrel count/size/placement, band count, lighting direction,
contact shadow, exposure, contrast, blur, sensor noise, JPEG artefacts. 55% of
barrels are placed beside a painted line — the case that matters.

Measured on 30 held-out frames where *every* barrel sits on a painted line:

| metric | result |
|---|---|
| barrel pixels predicted `cone` | 94.8% |
| barrel pixels leaking into `lane` | 1.1% |
| val mask mAP50 | 0.858 |

Synthetic frames come from a compositor, not ogre2, so shading and shadows
differ from Gazebo. Mixing in a few hundred real captured frames
(`capture_frames.py`) closes that gap — but the synthetic set alone is enough
to answer "does the lane/cone separation work".

## Flexible class scheme (build full, trim later)
Train on all three classes (`lane, cone, drivable`). The model always outputs
every class; the node just publishes the active ones. Switch with no retrain:
```bash
python3 lane_seg_node.py --mode full        # lane + cone + drivable
python3 lane_seg_node.py --mode lane_cone   # lane + cone
python3 lane_seg_node.py --mode lane_only   # lane only — cones left to lidar
```
`synth_gen.py` emits only `lane` and `cone` by default (pass `--drivable` to add
the third), because `routing.drivable.publish_points` is already `false`. The
3-class head is kept so `mode: full` still loads.

## Pipeline at a glance
```
/head_camera/images ─► YOLOv8-seg ─┬─ lane mask  ─► ground-project ─► /lane/points ─► costmap "vision_lane" layer
                                   ├─ cone boxes ─► ground-project ─► /cone/points ─► costmap "vision_cone" layer
                                   └─ overlay ────────────────────► /lane/debug_image (RViz/Foxglove)
```

## The ground projection — already calibrated for the sim

`vision_config.yaml → projection` is **derived, not guessed**, from the xacro
chain. Do not "fix" these numbers without re-deriving them:

```
camera.xacro          chassis   -> camera_link   xyz = 0.8 0 0.315
robot_core_ref.xacro  base_link -> chassis       xyz = -0.965423 0 0
  => camera in base_link: x = -0.165423, z = 0.315
base_link rides on the WHEEL AXLE (wheel_radius 0.1625)
  => cam_height above ground = 0.315 + 0.1625 = 0.4775
640x480, hfov 1.089 rad, square pixels
  => fx = fy = (640/2)/tan(1.089/2) = 528.43, cx,cy = 320,240
```

Note `cam_x_offset` is **negative**: `base_link` sits at the chassis *front
edge* and the chassis extends backwards, so the camera is behind it. An earlier
config had `+0.80`, which put every projected lane point about a metre too far
forward.

Output frame is `base_link`, which matches `robot_base_frame: base_link` in
`nav2_params.yaml` and the URDF root link. (The repo README used to say
`base_chassis_link`; no such link exists in the sim.)

**On the real robot**: measure the optical centre's height above ground and any
down-pitch, and update `cam_height` / `cam_pitch_deg` / `cam_x_offset`.
Everything else carries over.


## Lane linking — why a marked costmap is not enough

A dashed marking, a barrel standing on the line, or worn paint all reach the
costmap as **separate blobs**. Nav2 has no idea they are one wall, so it will
happily plan a path through the hole between two dashes. Marking the cells is
not sufficient; the boundary has to be *continuous*.

`lane_link.py` groups the fragments that belong to one physical boundary, fits
a curve, and resamples it densely.

### The thing that made this work

The obvious design is to join fragment endpoints whose headings agree. That
was built first and measured at **0.17 recall**. The data says why:

```
median detected fragment length      0.50 m
same-line fragment heading disagreement, median   52 degrees
```

A stubby 0.5 m blob has no meaningful principal axis. Per-fragment direction
is simply not a usable signal. Its *position*, though, is reliable. So the
rewrite seeds a line from the longest fragment and **grows** it: fit a curve to
the group so far, absorb any fragment lying close to that curve, refit, repeat.
Direction only ever comes from the accumulated curve, which has real extent.

Recall went 0.17 → **0.92** with precision unchanged at **1.000**.

Extrapolation is throttled so a noisy seed cannot fire off across the frame: a
group may reach past its own ends by about its own current span, clamped into
`[min_extrap, max_gap]`. Reach grows as the evidence does.

### Measured

| | |
|---|---|
| pair precision (left+right never merged) | **1.000** (0 false merges) |
| pair recall (dashes of one line joined) | **0.921** |
| lane lines found per frame | 1.96 (2 is correct) |
| true lane covered, within detected span | **72.7% → 83.3%** |

Precision matters more than recall here. Under-linking leaves a gap Nav2 might
squeeze through; **over-linking invents a wall straight across the corridor**,
which would block a route that is actually open. The lateral tolerance is what
guarantees it: the opposite boundary sits a corridor-width away (2.4–4.4 m),
five-plus times the 0.50 m tolerance, so it can never be absorbed.

### Topics

| topic | what |
|---|---|
| `/lane/points` | **continuous**, gap-free — this is what Nav2 consumes |
| `/lane/points_raw` | the unlinked points, for comparison in RViz |
| `/lane/lines` | `MarkerArray`, one coloured `LINE_STRIP` per lane line |

The debug image also draws each reconstructed line in its own colour with its
fragment centroids dotted, so you can see at a glance which pieces were joined.

Tune it in `vision_config.yaml → linking`. Set `enabled: false` to go back to
raw points. Note `resample_step` must stay **below** the costmap resolution
(0.05 m in `nav2_params.yaml`) or a diagonal line skips cells and the
"continuous" wall gets holes again.

Run the verification yourself:
```bash
python3 test_lane_link.py ~/esda_models/lane_seg.pt 25
python3 viz_link.py       ~/esda_models/lane_seg.pt   # writes the figure
```

## Running on the GPU instead of the CPU

Four separate workloads, and they do NOT share a switch. Run `bash gpu_check.sh`
from the repo root — it reports each one and measures what GPU actually buys.

| workload | how it goes to the GPU | notes |
|---|---|---|
| Gazebo + RViz rendering | `./demo.sh up --gpu` | **the big one** — see below |
| YOLO inference | already on: `model.device: "0"` | add `model.half: true` for ~2x |
| Training | `train.py --device 0` | already the default |
| Fastest inference | TensorRT engine | `export_jetson.py`, build it on this box |

### Gazebo rendering is the real CPU hog

`LIBGL_ALWAYS_SOFTWARE=1` is set in `demo.sh`, `run.sh` and `~/.bashrc`. It
forces llvmpipe, so **every camera and lidar frame is rasterised on the CPU**.
At 640x480 with the sensor plugin running, that dominates everything else in
this project — more than the YOLO node.

`./demo.sh up --gpu` switches to hardware OpenGL:

```
LIBGL_ALWAYS_SOFTWARE=0
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
LD_LIBRARY_PATH=/usr/lib/wsl/lib:...
```

It is **not** the default, deliberately. ogre2 on the WSL D3D12 mapping layer
has a history of killing the Gazebo sensors mid-run, and a dead `/scan` is a
worse problem than a busy CPU — that is what the original
"software rendering keeps Gazebo sensors from crashing in WSL" comment was
about. `--gpu` prints the renderer it resolved before launching, and if the sim
or `/scan` fails to appear it tells you to fall back. Try it; if it is stable
on your driver, keep using it.

### YOLO: already on the GPU, but not in FP16

`vision_config.yaml` already has `model.device: "0"`, so the node uses CUDA
when torch can see it (and falls back to CPU with a warning when it cannot).
FP16 is the free ~2x on top:

```bash
# test without editing anything
python3 vision/lane_seg_node.py --device 0 --half
# then make it permanent in vision_config.yaml:  model.half: true
```

Also available: `--device cpu` to force CPU, `--no-half`, and `--no-link` to
turn lane linking off.

For reference, measured on 2 CPU cores at imgsz 640: **140 ms/frame (7 fps)**.
The node only needs 5 Hz, so CPU *just* keeps up — but only if nothing else
wants the CPU, which is exactly not the case when Gazebo is software-rendering.

### Not a GPU switch, but the other big lever

`worlds/igvc.sdf` uses `max_step_size 0.001` — 1000 physics steps per second.
Raising it to `0.002` roughly halves physics CPU, at some fidelity cost. Worth
trying if the sim still runs below real time after moving rendering to the GPU.
