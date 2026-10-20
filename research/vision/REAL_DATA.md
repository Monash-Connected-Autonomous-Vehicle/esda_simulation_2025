# Real photographs: what to take, and why

One model for Gazebo and the real vehicle. That is achievable, but the real
half of the dataset is the half only you can collect, and the *useful* photo
set looks different from the obvious one.

## The one thing to get right

**Photograph from the robot's camera, at the robot's camera height, through
the robot's camera.** 0.4775 m above the ground, 640x480, 62° horizontal FOV,
looking level. A phone held at chest height is a different sensor looking at
a different scene, and the model will learn the phone.

The cheapest way is the robot itself:

```bash
# on the vehicle, camera node running
python3 vision/capture_frames.py /head_camera/images 0.5 ~/esda_real
```

Walk it round the course. If the real camera is not ready, a phone **on the
ground, lens at 48 cm, held level** is a reasonable stand-in — tape it to a
box of the right height and keep it level.

## What to shoot

Aim for **300–600 frames**. Below ~200 the real domain cannot hold its own
against thousands of sim frames; above ~600 the labelling becomes the whole
weekend.

Spend them on *variety*, not quantity:

| Spend | On |
|---|---|
| 30% | **Hard shadows.** Tree shadows, building edges, the robot's own shadow, sun low and across the lane. This is the measured failure — the renderer never produced one and the model has never seen one. |
| 20% | Lines at distance and at an angle — approaching a bend, line leaving frame |
| 15% | Barrels: alone, in pairs, partly out of frame, one occluding another |
| 15% | Bad light: overcast, dusk, glare straight into the lens, wet tarmac |
| 10% | Lines that are worn, dirty, patched, interrupted |
| 10% | **Negatives** — tarmac with no line at all, kerbs, drain covers, white litter, painted markings that are NOT the lane |

That last row matters more than it looks. The model's failure is a *false
positive* on shadow, so it needs to be shown things that look like paint and
are not.

## Labelling

Polygons, YOLOv8-seg format, three classes in this order:

```
0 lane      the painted boundary line itself, not the area between
1 cone      construction barrel / traffic cone, the whole visible silhouette
2 drivable  tarmac you could drive on, inside the lines, minus the barrels
```

Use Roboflow, CVAT or label-studio; export "YOLOv8 segmentation". Keep the
class order — it must match `classes.names` in `vision_config.yaml`.

Rule of thumb for consistency, which matters more than precision: **label the
shadow as drivable, never as lane.** Every frame. That single convention is
what teaches the thing we need taught.

## Merging with the sim data

```bash
# 1. thousands of auto-labelled real-renderer frames, no manual work
python3 vision/autolabel.py --capture ~/esda_capture/igvc_oval_test \
                            --out ~/esda_datasets/real_sim --overlay
#    LOOK AT ~/esda_datasets/real_sim/overlay/*.jpg FIRST

# 2. your photographs
#    ~/esda_datasets/real_photos/{images,labels}/{train,val}

# 3. merge, oversampling the real ones so they are not drowned out
python3 vision/mix_datasets.py --sim  ~/esda_datasets/real_sim \
                               --real ~/esda_datasets/real_photos \
                               --out  ~/esda_datasets/mixed --balance 0.35

# 4. train, then score the two domains SEPARATELY
python3 vision/train.py --data ~/esda_datasets/mixed/data.yaml --epochs 120
yolo segment val model=runs/segment/lane_seg/weights/best.pt \
                 data=~/esda_datasets/mixed/data_val_sim.yaml
yolo segment val model=runs/segment/lane_seg/weights/best.pt \
                 data=~/esda_datasets/mixed/data_val_real.yaml
```

A single merged mAP is worthless: 95% could be 97% sim and 40% real, and you
would ship it. Always quote both numbers.

## What to expect

Sim mAP will dip slightly when real data joins — that is the model giving up
some overfitting to the renderer, and it is the trade you want. The number to
watch is real mAP, and the honest test is still the lap benchmark on a
held-out world, not the validation score.
