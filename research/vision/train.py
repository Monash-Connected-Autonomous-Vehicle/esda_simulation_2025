#!/usr/bin/env python3
"""
Train the ESDA lane/cone segmentation model (YOLOv8-seg) on the GPU server.

Sim-first: train on the RTX / server GPU, validate against the Gazebo camera.
Build with all classes (lane, cone, drivable); you can still run inference in
lane_only mode later without retraining (see vision_config.yaml).

Usage (ROS not required for training):
    pip install ultralytics
    python3 train.py                                  # defaults below
    python3 train.py --data ~/esda_datasets/lane_seg/data.yaml \
                     --model yolov8s-seg.pt --epochs 120 --imgsz 640 --device 0
    python3 train.py --model yolov8n-seg.pt           # smaller/faster (Jetson)

Model size guide:
    yolov8n-seg  - smallest, best for Jetson Nano real-time
    yolov8s-seg  - good sim/server default (accuracy vs speed)
    yolov8m-seg  - most accurate, train on server, probably too heavy on Jetson

Output: runs/segment/<name>/weights/best.pt  -> copy to ~/esda_models/lane_seg.pt
"""
import argparse
import os
import shutil
import sys


def _resolve_data_path(data):
    """Return a data yaml whose `path:` is absolute, writing a sibling copy if
    the original uses ~ or a relative path."""
    try:
        import yaml
    except ImportError:
        return data
    with open(data) as f:
        cfg = yaml.safe_load(f) or {}
    raw = cfg.get("path")
    if not raw:
        return data
    absolute = os.path.abspath(os.path.expanduser(str(raw)))
    if str(raw) == absolute:
        return data
    cfg["path"] = absolute
    out = os.path.join(os.path.dirname(os.path.abspath(data)), "data_resolved.yaml")
    with open(out, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"[train] dataset path '{raw}' -> '{absolute}'  (wrote {out})")
    return out


def main():
    ap = argparse.ArgumentParser(description="Train ESDA lane/cone YOLOv8-seg")
    ap.add_argument("--data", default="~/esda_datasets/lane_seg/data.yaml",
                    help="dataset yaml (use Roboflow's export data.yaml if you have it)")
    ap.add_argument("--model", default="yolov8s-seg.pt",
                    help="base weights: yolov8n-seg.pt | yolov8s-seg.pt | yolov8m-seg.pt")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16, help="-1 for auto")
    ap.add_argument("--device", default="0", help="'0' GPU, 'cpu', or '0,1'")
    ap.add_argument("--name", default="lane_seg")
    ap.add_argument("--patience", type=int, default=30, help="early-stop patience")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--export", default="~/esda_models/lane_seg.pt",
                    help="where to copy best.pt after training")
    args = ap.parse_args()

    data = os.path.expanduser(args.data)
    if not os.path.isfile(data):
        sys.exit(f"[train] data yaml not found: {data}\n"
                 f"        Generate one with  python3 synth_gen.py --out <dir>  (no\n"
                 f"        labelling needed), export a dataset from Roboflow in YOLOv8\n"
                 f"        format, or lay out images/{{train,val}} + labels/{{train,val}}\n"
                 f"        and point --data at it.")

    # Ultralytics resolves a dataset `path:` that is not absolute against its
    # OWN datasets dir, not the cwd -- so a yaml saying `path: ~/esda_datasets/
    # lane_seg` silently looks in <datasets_dir>/~/esda_datasets/lane_seg and
    # reports "no images found". Rewrite it to an absolute path first.
    data = _resolve_data_path(data)

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("[train] ultralytics not installed.  pip install ultralytics")

    # If the user passed a bare name like "yolov8s-seg.pt", prefer a copy we
    # already have. fetch_deps.sh stages these into ~/esda_models/base so a
    # first training run does not need the network at all.
    base = args.model
    if os.path.dirname(base) == "":
        staged = os.path.expanduser(os.path.join("~/esda_models/base", base))
        if os.path.isfile(staged):
            print(f"[train] using pre-staged base weights: {staged}")
            base = staged

    print(f"[train] base={base}  data={data}  epochs={args.epochs}  device={args.device}")
    model = YOLO(base)
    results = model.train(
        data=data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        name=args.name,
        patience=args.patience,
        resume=args.resume,
        # light augmentation suited to a ground-camera lane view
        degrees=5.0, translate=0.1, scale=0.3, fliplr=0.5, mosaic=1.0,
    )

    # locate best.pt and copy to the serving location
    save_dir = getattr(results, "save_dir", None) or model.trainer.save_dir
    best = os.path.join(str(save_dir), "weights", "best.pt")
    if os.path.isfile(best):
        dst = os.path.expanduser(args.export)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(best, dst)
        print(f"\n[train] done. best weights -> {dst}")
        print(f"[train] point vision_config.yaml model.weights at: {dst}")
    else:
        print(f"\n[train] done, but best.pt not found under {save_dir}")


if __name__ == "__main__":
    main()
