# Running the YOLO lane + cone detector

The weight is in the repo. Nothing to download, nothing to train.

    research/models/lane_seg_synth_cpu.pt    # yolov8n-seg, 3 classes, 27 MB

Three classes: **lane**, **cone**, **drivable**. This is the only detector we
have that separates cones from lane paint — the classic thresholds are
lane-only by construction, which is the whole reason this model exists. A
white-striped barrel read as lane paint inflates the costmap into a false
wall across the drivable gap.

---

## 1. From the UI

Launch the simulation, then pick **YOLO seg (lane+cone)** in the lane
detector dropdown and press the lane detection button. That is it.

The UI passes the camera mount it launched the sim with, so the projection
matches the robot that is actually standing there.

**While it is selected, Nav2 drives on the raw LiDAR.** See the Nav2 note
below — this is deliberate, and it is the one thing to understand before
reading a lap result.

## 2. From a launch file

```bash
pip install ultralytics          # once; the only extra dep
ros2 launch esda_simulation_2025 lane_seg.launch.py
```

| argument | default | |
|---|---|---|
| `camera_mount` | `pole_top` | **must match the mount the sim was launched with**; the projection is derived from it |
| `detector` | `yolo` | `classic` for the threshold instead |
| `mode` | `full` | `lane_cone`, `lane_only` to trim compute |
| `device` | config | `0` for the first GPU, `cpu` to force CPU |
| `image_topic` | `/camera/left/image_raw` | |
| `scan_topic` | `/scan` | raw LiDAR, for the cone gate |

Geometry comes from the xacro chain, not from a measurement anyone took
with a ruler: `pole_top` is 1.4625 m above the ground, 15° down, 0.405 m
behind `base_link`; `front` is 0.4775 m, level, 0.165 m behind. Getting
this wrong fails **silently** — points still come out, just in the wrong
place.

## 3. By hand, from the research folder

```bash
cd research/vision
python3 lane_seg_node.py --detector yolo
```

`model.weights` in `vision_config.yaml` is a relative path resolved against
the config file, so it finds the committed weight from any working
directory. Point it at an absolute path or `~/esda_models/...` to use your
own instead. Run from the installed package instead and it picks up
`share/esda_simulation_2025/vision/vision_config.yaml` on its own.

| flag | what it does |
|---|---|
| `--detector yolo` \| `classic` | network, or the threshold. Overrides the config. |
| `--mode full` \| `lane_cone` \| `lane_only` | which classes get published. No retrain needed — the model still outputs all three, muted ones just aren't published. |
| `--device cpu` \| `0` | falls back to cpu on its own if CUDA isn't visible. |
| `--half` | FP16. Jetson/TensorRT only. |
| `--no-link` | turn off lane linking, to see the raw fragments. |

Topics to watch:

| topic | what |
|---|---|
| `/lane/points` | linked, gap-free lane boundary → this is what Nav2 consumes |
| `/lane/points_raw` | the same before linking, for comparison |
| `/cone/points` | confirmed cones, as filled discs of barrel radius |
| `/lane/lines` | `MarkerArray`, one coloured strip per lane line |
| `/lane/debug_image` | annotated overlay for RViz/Foxglove |

**Cones are lidar-gated.** A vision cone is only marked where a lidar return
agrees within `cone_gate.confirm_radius` (0.60 m). A vision guess alone does
not become an obstacle — it has already fired on cast shadows and a black
board. Set `cone_gate.enabled: false` to see the ungated detections, but
don't drive on them.

## 4. The sim's own lane node — `lane_detection_esda.py`

Lane only, no cones, but it's the one the UI and the launch files already
use. It now takes `yolo` as a method:

```bash
ros2 run esda_simulation_2025 lane_detection_esda.py \
  -p detector_method:=yolo
```

`yolo_weights` defaults to the committed weight; pass it to override, or set
`ESDA_YOLO_WEIGHTS`. If the weight or `ultralytics` is missing it logs the
reason and falls back to `adaptive` rather than dying mid-run.

---

## Nav2 does not consume any of this yet

The stack plans on `/scan_fused`, a LaserScan that `lane_detection.py` and
`lane_detection_esda.py` publish with lane obstacles already fused into it.
This node publishes point clouds instead and no `/scan_fused`, so while it
is the selected detector the UI points Nav2 back at the raw `/scan`.

That means: **a lap run with this detector is a lap run on LiDAR only.** The
lanes and cones are published, drawn and scoreable, but nothing plans around
them. Do not read a distance from such a run as a verdict on the detector.

Wiring the clouds in is a separate, deliberate change —
`nav2_vision_costmap_snippet.yaml` in this folder has the observation-source
block for `/lane/points` and `/cone/points`. It wants a scored run before
anyone relies on it, because false lane pixels become lethal cells and we
have walled the robot in with those once already.

---

## Before you read the output as a verdict

On our 395-frame teleport bench **in simulation**, the network lost to every
threshold on every axis:

| detector | recall | precision | IoU | ms/frame |
|---|---|---|---|---|
| yolo | 0.492 | 0.362 | 0.263 | 8.4 |
| otsu | 0.800 | 0.452 | 0.406 | 0.9 |
| adaptive | 0.795 | 0.263 | 0.247 | 2.4 |

That is why `detector.kind` still defaults to `classic`.

Three honest caveats, because that table is not the last word:

- **It was trained on synthetic frames**, on CPU, from `synth_gen.py` renders —
  not on photographs of the real course. It is being scored on roughly its
  training distribution and still losing, which is a real result, but a model
  trained on a few hundred real labelled frames is a different model.
- **The bench is uniformly lit.** This sim has no shadows worth the name, and
  shadow robustness is the one axis where a learned model should beat a
  threshold. Job 110d had to *synthesise* shadow bands to probe it.
- **Cones aren't in that table at all.** The thresholds score 0 there, because
  they cannot do it. If what you need is cone/lane separation, the comparison
  above is not the comparison you care about.

So: worth trying on real footage, and the only option if you want cones from
the camera. Watch **precision** more than recall — a false lane pixel becomes
a lethal costmap cell, and we have walled the robot in with those once.

## Retraining it

```bash
python3 synth_gen.py --out ~/esda_datasets/lane_seg --n-train 1600
python3 train.py --data ~/esda_datasets/lane_seg/data.yaml   # downloads the base weight
```

`train_cpu.py` is the no-GPU path that produced the committed weight.
`export_jetson.py` converts a `.pt` to a TensorRT `.engine` for the Orin.
`eval_recall.py --detector yolo` reproduces the table above.
