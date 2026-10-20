# ESDA Vision — Runbook for the server-side Claude

Goal: a trained lane/cone segmentation model running live against the Gazebo
camera, feeding Nav2 through two separate costmap layers. Work top to bottom.
Stop and report if a VERIFY step fails.

**Context.** The classical white-line detector tagged white-striped barrels as
*lane*, inflating the Nav2 costmap into a false wall the robot could not pass.
The fix is a YOLOv8-seg model with **separate `lane` and `cone` classes on
separate topics**, feeding different costmap layers.

This is not a tuning problem. Sampling the sim's own textures gives:

| thing | colour (BGR) |
|---|---|
| lane paint, `ground.png` | 242, 242, 242 |
| barrel reflective band, `Construction_Barrel_Diffuse.png` | 235, 224, 216 |

They are the same colour. **No threshold can separate them** — the model has to
use shape and context. That is the whole justification for this pipeline.

Build with all classes (`lane, cone, drivable`); the inference node can be
switched to `lane_only` later without retraining if compute is tight (lidar
already sees cones).

> **What changed in this revision.** Task 2 used to be "label 200-400 frames in
> Roboflow" — hours of manual polygon work, and the thing that blocked this
> pipeline for weeks. It is now optional. `synth_gen.py` renders labelled
> frames directly from the sim's own textures using the exact camera model, so
> labels are pixel-exact and free. A model trained purely on synthetic frames
> already scores **mask mAP50 0.858**, with **94.8%** of barrel pixels called
> `cone` and only **1.1%** leaking into `lane` on worst-case frames where every
> barrel sits on a painted line.

---

## Task 0 — Environment (once)

```bash
cd ~/esda_ws/src/esda_simulation_2025/vision   # or wherever vision/ landed
bash setup_server.sh
```
VERIFY (new terminal):
```bash
ros2 pkg list | grep esda_simulation_2025
python3 -c "import torch; print('cuda', torch.cuda.is_available())"   # must be True
ros2 launch esda_simulation_2025 launch_sim_headless.launch.py        # starts clean
```
If `cuda` is False but the box has a GPU:
`pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121`

---

## Task 1 — Get a dataset

### 1a. Synthetic (recommended, ~4 minutes, no Gazebo and no labelling)

```bash
cd ~/esda_ws/src/esda_simulation_2025/vision
python3 synth_gen.py --out ~/esda_datasets/lane_seg \
                     --n-train 1600 --n-val 280 --preview 8
```

It renders the camera view from `ground.png` and
`Construction_Barrel_Diffuse.png` using the same ground-plane homography that
`lane_seg_node.py` uses for projection, so training and deployment share one
geometry. Randomised per frame: lane offset, heading, curvature, corridor
width, paint width, dashes, barrel count/size/placement, band count, lighting,
shadow, exposure, blur, noise, JPEG artefacts. Barrels are deliberately placed
beside a painted line 55% of the time — the hard case.

It writes its own `data.yaml` with an **absolute** `path:`. This matters:
ultralytics resolves a relative or `~` path against its *own* datasets dir, not
your cwd, and then reports "no images found".

VERIFY:
```bash
ls ~/esda_datasets/lane_seg/images/train | wc -l     # ~1600
cat ~/esda_datasets/lane_seg/data.yaml               # path: is absolute
xdg-open ~/esda_datasets/lane_seg/preview            # labels sit on the right pixels
```
In the previews, green must cover only painted lines and orange must cover each
whole barrel **including its white bands**. If a white band is green, stop —
that is the bug this pipeline exists to prevent.

### 1d. More arenas -- the best source of extra in-domain data

The synthetic generator varies lane geometry, but it is a compositor, not
ogre2. It cannot teach the model what real Gazebo shading, shadows and barrel
geometry look like. Extra ARENAS can, because they are rendered by the same
engine you deploy against:

```bash
python3 make_worlds.py --list
python3 make_worlds.py --worlds all --preview     # writes into the package
bash ../setup_wsl.sh --sync                        # install the new files
./demo.sh up --world igvc_oval                     # or igvc_wavy / igvc_figure8 /
                                                   #    igvc_slalom / igvc_spline
```

Each arena is a new course drawn onto the REAL ground.png asphalt (its
original painted lines inpainted out), a randomised barrel field with barrels
straddling the lines, and a randomised sun direction and intensity. Barrels are
kept at least 3 m from the robot spawn at the origin, so nothing spawns inside
the robot.

Cheap on disk: the floor is one 43x37 m quad with one texture, so a world is
just a PNG plus a 3 KB .dae. Barrels reference the existing
`construction_barrel.dae` by package:// URI -- no meshes are duplicated and no
new model directories are created.

Drive each arena, capture with option 6, split each run with `split_dataset.py`
(blocks, per run), and concatenate. Five layouts x a few hundred frames is far
more useful than five times as many frames from one lap, because the variation
is in the thing you actually want the model to be invariant to.

### 1c. Splitting real frames honestly

Training on the arena you will actually run in is **not** overfitting -- it is
domain specialisation, and for passing the sim test it is the right call.
Overfitting is narrower: memorising the training IMAGES rather than the
concept. You can do that inside one arena, and the usual way it slips past you
is the train/val split.

`capture_frames.py` saves a frame every N seconds while the robot drives, so
frame t and t+1 are nearly the same picture. Shuffle and split 80/20 and
near-duplicates land on both sides; the val score then measures recall of
pictures the model already saw.

```bash
# how leaky would a random split be on YOUR frames? measures, writes nothing
python3 split_dataset.py --images ~/lane_dataset --report-only

# build a split that holds out contiguous stretches of the run instead
python3 split_dataset.py --images ~/lane_dataset --labels ~/lane_dataset/labels \
                         --out ~/esda_datasets/arena --val-frac 0.2 --blocks 4
```

It reports consecutive-frame similarity and the leakage a random split would
cause, then holds out several contiguous chunks spread through the run so val
frames come from course the model never trained on. If it says the difference
is small, believe it -- the tool does not manufacture alarm.

Keep the synthetic set mixed in. It is the only part of the data where lane
geometry, barrel placement and lighting vary, so it is what stops the model
learning "the lane is always about here in the frame" -- which matters the day
the barrels move, or on the real robot.

### 1b. Real Gazebo frames (worth adding on top)

Synthetic frames are rendered by a compositor, not by ogre2, so shading and
shadows differ. A few hundred real frames mixed in closes that gap.

```bash
# terminal A
ros2 launch esda_simulation_2025 launch_sim_headless.launch.py
# terminal B
python3 ~/esda_ws/src/esda_simulation_2025/capture_frames.py /head_camera/images 1.0 ~/lane_dataset
# terminal C — drive to vary the view
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Label those in Roboflow (Instance Segmentation; classes **in this exact order**:
`lane`, `cone`, `drivable`), export YOLOv8, and copy the images/labels into the
synthetic dataset's `images/train` and `labels/train`.

> On pretrained Roboflow models: the team already found one scored **0 objects**
> on this world. Treat an off-the-shelf real-world dataset as extra volume, not
> as the base — the domain gap to a Gazebo IGVC course is large.

---

## Task 2 — Train (GPU)

```bash
cd ~/esda_ws/src/esda_simulation_2025/vision
python3 train.py --data ~/esda_datasets/lane_seg/data.yaml \
                 --model yolov8s-seg.pt --epochs 120 --imgsz 640 --device 0
```
- `yolov8s-seg` is the sim/server default. `yolov8n-seg` if you'll push to a
  Jetson; `yolov8m-seg` for max accuracy on the server.
- Script copies `best.pt` → `~/esda_models/lane_seg.pt` automatically.

VERIFY: `mAP50(M)` for `lane` and `cone` both reasonable (>0.6 is a good start;
the synthetic-only CPU baseline hit 0.858), and `~/esda_models/lane_seg.pt`
exists. If lane and cone get confused, generate more frames — `synth_gen.py`
already biases barrels toward lane lines, so just raise `--n-train`.

---

## Task 3 — Run it live against the sim

```bash
# sim already running, else launch it
python3 ~/esda_ws/src/esda_simulation_2025/vision/lane_seg_node.py
ros2 run rqt_image_view rqt_image_view /lane/debug_image
ros2 topic hz /lane/points
ros2 topic hz /cone/points
```
VERIFY: `/lane/debug_image` shows lanes green and barrels orange, **distinct**;
`/lane/points` has data on lane frames and `/cone/points` on cone frames.

**Check the cone count.** Put 3+ barrels in view and confirm `/cone/points`
carries roughly one cluster per barrel. Two bugs that used to collapse this to
a single cone are fixed, but it is the thing most worth re-checking on real
output:
- the box-base fallback was gated on the cone list being empty, so only the
  first mask-less cone was ever projected;
- `_project()` applied the `row_stride`/`col_stride` subsample to those sparse
  contact points, deleting ~15 of every 16.

To trim compute: `python3 lane_seg_node.py --mode lane_only`.

**Check the lane lines are CONTINUOUS.** A dashed marking or a barrel standing
on a line reaches the costmap as separate blobs, and Nav2 will plan straight
through the holes between them. `lane_link.py` rebuilds each boundary as one
line, and the node now publishes:

| topic | what |
|---|---|
| `/lane/points` | continuous, gap-free -> what Nav2 consumes |
| `/lane/points_raw` | the unlinked points, for comparison |
| `/lane/lines` | `MarkerArray`, one coloured line per lane boundary |

```bash
ros2 topic echo /lane/lines --once      # one LINE_STRIP per boundary
```
In RViz add both `/lane/points_raw` (gappy) and `/lane/points` (continuous)
and compare. On a two-boundary course you want **two** markers, not one and
not five. One marker spanning both boundaries would be an over-merge, and that
is the dangerous failure -- it invents a wall across a corridor that is open.

Run the linker's own tests any time:
```bash
python3 test_lane_link.py ~/esda_models/lane_seg.pt 25
```

**If the projected points land at the wrong distance**, re-check the
`projection` block of `vision_config.yaml` against the xacro chain rather than
guessing. For the sim it is already correct and derived:
`cam_height 0.4775` (0.315 mount + 0.1625 wheel radius, because `base_link`
rides on the axle) and `cam_x_offset -0.165423` (base_link sits at the chassis
*front edge*, so the camera is behind it). On the real robot, measure.

---

## Task 4 — Feed Nav2 (the actual fix)

Merge `nav2_vision_costmap_snippet.yaml` into `config/nav2_params.yaml`,
rebuild, relaunch Nav2.

**Read the snippet's header first.** Its `plugins:` list must keep
`voxel_layer` — that is this repo's only lidar source, and an earlier version
of the snippet would have replaced it with a non-existent `obstacle_layer`,
failing Nav2 configure and blinding the robot to `/scan`.

VERIFY:
```bash
ros2 param get /local_costmap/local_costmap plugins   # must still list voxel_layer
```
Note `/lane/points` is now the LINKED, continuous cloud, so the lane layer gets
an unbroken boundary instead of a dotted one. Keep `linking.resample_step`
below the costmap `resolution` (0.05) or the raster skips cells.
and in RViz the local costmap shows lane lines as a thin boundary (not a solid
block over the barrels), with the robot planning *between* barrels.

---

## Report back to the hardware/laptop Claude
- dataset size + class balance (synthetic vs real split)
- final mAP50 per class
- path to `~/esda_models/lane_seg.pt`
- `/lane/debug_image` screenshot + `/lane/points` & `/cone/points` hz
- cone count sanity: N barrels in view → N clusters on `/cone/points`
- whether Task 4 cleared the cone-as-lane costmap inflation
