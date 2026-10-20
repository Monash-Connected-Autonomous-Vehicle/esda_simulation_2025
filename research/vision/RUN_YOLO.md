# Running the YOLO lane + cone detector

The weight is in the repo. Nothing to download, nothing to train.

    research/models/lane_seg_synth_cpu.pt    # yolov8n-seg, 3 classes, 27 MB

Three classes: **lane**, **cone**, **drivable**. This is the only detector we
have that separates cones from lane paint — the classic thresholds are
lane-only by construction, which is the whole reason this model exists. A
white-striped barrel read as lane paint inflates the costmap into a false
wall across the drivable gap.

---

## 1. The cone detector — `lane_seg_node.py`

This is the node with cone detection. Lanes and cones go to **separate
topics**, so they can feed different costmap layers and can never be mixed.

```bash
# ROS 2 sourced, sim already running
pip install ultralytics                      # once; the only extra dep
cd research/vision
python3 lane_seg_node.py --detector yolo
```

That is the whole command. `model.weights` in `vision_config.yaml` is a
relative path resolved against the config file, so it finds the committed
weight from any working directory. Point it at an absolute path or
`~/esda_models/...` to use your own instead.

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

## 2. The sim's own lane node — `lane_detection_esda.py`

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
