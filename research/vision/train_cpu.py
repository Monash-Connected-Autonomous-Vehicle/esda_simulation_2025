import os, sys
os.environ.setdefault("OMP_NUM_THREADS","2")
import torch; torch.set_num_threads(2)
from ultralytics import YOLO
import argparse
ap=argparse.ArgumentParser()
ap.add_argument("--data", default="/home/claude/esda/dataset/data.yaml")
ap.add_argument("--model", default="yolov8n-seg.pt")
ap.add_argument("--epochs", type=int, default=40)
ap.add_argument("--imgsz", type=int, default=384)
ap.add_argument("--batch", type=int, default=16)
ap.add_argument("--name", default="lane_seg_cpu")
ap.add_argument("--fraction", type=float, default=1.0)
a=ap.parse_args()
m=YOLO(a.model)
m.train(data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, device="cpu",
        workers=2, name=a.name, patience=8, fraction=a.fraction, cache=False,
        val=True, plots=False, deterministic=False, seed=7,
        degrees=4.0, translate=0.08, scale=0.35, fliplr=0.5, mosaic=0.6,
        hsv_h=0.012, hsv_s=0.5, hsv_v=0.4, project="/home/claude/esda/runs")
