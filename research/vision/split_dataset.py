#!/usr/bin/env python3
"""
Split captured frames into train/val WITHOUT leaking near-duplicates, and
measure how much a random split would have leaked.

WHY THIS EXISTS
    Training on the arena you will actually run in is not overfitting -- it is
    domain specialisation, and it is the right call. Overfitting is a narrower
    thing: memorising the training IMAGES instead of learning the concept. You
    can do that inside a single arena, and the usual way people do it without
    noticing is the train/val split.

    capture_frames.py writes one frame every N seconds while the robot drives.
    Frame t and frame t+1 are nearly the same picture. Shuffle those and split
    80/20 and near-duplicates land on BOTH sides. The val score then measures
    how well the model recalls pictures it already saw, which is high, and you
    conclude it is working.

    Splitting by CONTIGUOUS BLOCKS of the run instead means the held-out
    frames come from stretches of course the model never trained on. Lower
    score, but it is the true one.

USAGE
    # measure only -- how bad would a random split be on YOUR frames?
    python3 split_dataset.py --images ~/lane_dataset --report-only

    # build an honest split (images + YOLO labels, if labels exist)
    python3 split_dataset.py --images ~/lane_dataset --labels ~/lane_dataset/labels \
                             --out ~/esda_datasets/arena --val-frac 0.2 --blocks 4

    # merge on top of a synthetic set so you keep the generaliser
    python3 split_dataset.py --images ~/lane_dataset --labels ~/lane_dataset/labels \
                             --out ~/esda_datasets/lane_seg --append

NOTE ON ORDERING
    Frames are ordered by filename, which is what capture_frames.py's
    frame_YYYYmmdd_HHMMSS_NNNN naming gives you: capture order. If your files
    are named differently, pass --sort mtime.
"""
import argparse
import os
import random
import shutil
import sys

import cv2
import numpy as np

IMG_EXT = (".jpg", ".jpeg", ".png", ".ppm")


def list_frames(d, how="name"):
    fs = [f for f in os.listdir(d) if f.lower().endswith(IMG_EXT)]
    if how == "mtime":
        fs.sort(key=lambda f: os.path.getmtime(os.path.join(d, f)))
    else:
        fs.sort()
    return fs


def thumb(path, w=80, h=60):
    im = cv2.imread(path)
    if im is None:
        return None
    g = cv2.cvtColor(cv2.resize(im, (w, h)), cv2.COLOR_BGR2GRAY).astype(np.float32)
    g -= g.mean()
    n = np.linalg.norm(g)
    return g / n if n > 1e-9 else g


def similarity_matrix(thumbs):
    M = np.stack([t.ravel() for t in thumbs])
    return M @ M.T          # already unit-normalised, so this is correlation


def leakage(sim, tr, va, thresh):
    if not len(va) or not len(tr):
        return 0.0
    sub = sim[np.ix_(va, tr)]
    return 100.0 * float((sub.max(axis=1) >= thresh).mean())


def block_split(n, val_frac, blocks, seed=0):
    """Hold out `blocks` contiguous chunks spread through the run.

    Several chunks rather than one tail chunk, so val covers several parts of
    the course instead of only the last stretch.
    """
    n_val = max(1, int(round(n * val_frac)))
    per = max(1, n_val // blocks)
    rng = random.Random(seed)
    starts, used = [], set()
    seg = n / blocks
    for b in range(blocks):
        lo = int(b * seg)
        hi = max(lo + 1, int((b + 1) * seg) - per)
        for _ in range(40):
            s = rng.randrange(lo, max(lo + 1, hi))
            if not any(abs(s - u) < per for u in used):
                starts.append(s)
                used.add(s)
                break
    val = set()
    for s in starts:
        val.update(range(s, min(n, s + per)))
    train = [i for i in range(n) if i not in val]
    return train, sorted(val)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True, help="folder of captured frames")
    ap.add_argument("--labels", default=None,
                    help="folder of YOLO .txt labels (same stems). Omit to split images only.")
    ap.add_argument("--out", default=None, help="dataset root to write")
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--blocks", type=int, default=4,
                    help="number of contiguous held-out chunks")
    ap.add_argument("--thresh", type=float, default=0.95,
                    help="similarity at or above which two frames are near-duplicates")
    ap.add_argument("--sort", choices=["name", "mtime"], default="name")
    ap.add_argument("--report-only", action="store_true")
    ap.add_argument("--append", action="store_true",
                    help="add to an existing dataset instead of replacing it")
    ap.add_argument("--copy", action="store_true",
                    help="copy files instead of symlinking")
    a = ap.parse_args()

    img_dir = os.path.expanduser(a.images)
    if not os.path.isdir(img_dir):
        sys.exit(f"no such folder: {img_dir}")
    frames = list_frames(img_dir, a.sort)
    n = len(frames)
    if n < 10:
        sys.exit(f"only {n} frames in {img_dir} -- capture more before splitting")
    print(f"[split] {n} frames in {img_dir} (ordered by {a.sort})")

    print("[split] measuring frame-to-frame similarity...")
    ths = []
    for f in frames:
        t = thumb(os.path.join(img_dir, f))
        if t is None:
            sys.exit(f"could not read {f}")
        ths.append(t)
    sim = similarity_matrix(ths)
    np.fill_diagonal(sim, -1.0)

    adj = [float(sim[i, i + 1]) for i in range(n - 1)]
    print(f"        consecutive frames: mean similarity {np.mean(adj):.3f}  "
          f"(max {np.max(adj):.3f})")
    near = sum(1 for v in adj if v >= a.thresh)
    print(f"        {near}/{n-1} consecutive pairs are near-duplicates "
          f"(>= {a.thresh})")

    ids = list(range(n))
    sh = ids[:]
    random.Random(0).shuffle(sh)
    cut = int((1 - a.val_frac) * n)
    rnd_tr, rnd_va = sh[:cut], sh[cut:]
    blk_tr, blk_va = block_split(n, a.val_frac, a.blocks)

    lr = leakage(sim, rnd_tr, rnd_va, a.thresh)
    lb = leakage(sim, blk_tr, blk_va, a.thresh)
    print()
    print("  split      val frames   val frames with a near-twin in train")
    print(f"  random     {len(rnd_va):>10}   {lr:>5.1f}%   <- inflates your score")
    print(f"  block      {len(blk_va):>10}   {lb:>5.1f}%   <- the honest one")
    print()
    if lr - lb < 2.0:
        print("  Little difference here: your frames are far enough apart that a")
        print("  random split is not badly leaky. Block split is still the safer")
        print("  default, and costs nothing.")
    else:
        print(f"  A random split would have reported a val score inflated by"
              f" {lr:.0f}% of frames the model had effectively already seen.")

    if a.report_only or not a.out:
        print("\n[split] report only, nothing written")
        return

    out = os.path.expanduser(a.out)
    lbl_dir = os.path.expanduser(a.labels) if a.labels else None
    place = shutil.copy2 if a.copy else os.symlink

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        d = os.path.join(out, sub)
        if os.path.isdir(d) and not a.append:
            shutil.rmtree(d)
        os.makedirs(d, exist_ok=True)

    missing = 0
    for split, idxs in (("train", blk_tr), ("val", blk_va)):
        for i in idxs:
            f = frames[i]
            stem = os.path.splitext(f)[0]
            src_i = os.path.abspath(os.path.join(img_dir, f))
            dst_i = os.path.join(out, "images", split, f)
            if not os.path.exists(dst_i):
                place(src_i, dst_i)
            if lbl_dir:
                src_l = os.path.join(lbl_dir, stem + ".txt")
                if os.path.isfile(src_l):
                    dst_l = os.path.join(out, "labels", split, stem + ".txt")
                    if not os.path.exists(dst_l):
                        place(os.path.abspath(src_l), dst_l)
                else:
                    missing += 1

    print(f"\n[split] wrote {out}")
    print(f"        train {len(blk_tr)}   val {len(blk_va)}   "
          f"({'copied' if a.copy else 'symlinked'})")
    if missing:
        print(f"        WARNING: {missing} frames had no label file -- "
              f"ultralytics treats those as background (no objects)")

    yml = os.path.join(out, "data.yaml")
    if not os.path.isfile(yml) or not a.append:
        with open(yml, "w") as fh:
            fh.write(f"# written by split_dataset.py (block split, no leakage)\n"
                     f"path: {out}\ntrain: images/train\nval: images/val\n\n"
                     f"names:\n  0: lane\n  1: cone\n  2: drivable\n")
        print(f"        data.yaml -> {yml}")


if __name__ == "__main__":
    main()
