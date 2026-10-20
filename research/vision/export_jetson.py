#!/usr/bin/env python3
"""
Export the trained .pt to a TensorRT engine for fast inference.

Despite the name this is NOT Jetson-only. It is just as useful on the WSL
desktop GPU: a TensorRT engine is typically ~2-3x faster than the .pt, which
matters here because every millisecond the vision node does not spend is a
millisecond Gazebo gets back.

THE ONE RULE: build the engine ON THE MACHINE THAT WILL RUN IT. TensorRT
engines are tied to the GPU architecture and the TensorRT/driver version, so
an engine built on the WSL RTX will NOT load on a Jetson, and vice versa.
ONNX is portable; .engine is not.

    # on the WSL box, for the WSL box
    python3 export_jetson.py ~/esda_models/lane_seg.pt --format engine --half
    # then in vision_config.yaml:
    #   model.weights: ~/esda_models/lane_seg.engine
    #   model.half: true

    python3 export_jetson.py ~/esda_models/lane_seg.pt --format onnx    # portable
"""
import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("weights")
    ap.add_argument("--format", default="engine", choices=["engine", "onnx"])
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--half", action="store_true", help="FP16 (recommended on Jetson)")
    args = ap.parse_args()

    w = os.path.expanduser(args.weights)
    if not os.path.isfile(w):
        sys.exit(f"weights not found: {w}")
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("pip install ultralytics")

    YOLO(w).export(format=args.format, imgsz=args.imgsz, half=args.half, device=0)
    print(f"exported {args.format} next to {w}")


if __name__ == "__main__":
    main()
