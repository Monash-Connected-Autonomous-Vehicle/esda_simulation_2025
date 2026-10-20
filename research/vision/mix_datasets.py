#!/usr/bin/env python3
"""
mix_datasets.py -- merge sim and real datasets into ONE model's training set.

The goal is a single lane/cone model that works in Gazebo and on the real
vehicle, rather than two models and a switch. That works, but only if two
things are handled deliberately.

OVERSAMPLING. There will be thousands of auto-labelled sim frames and perhaps
a few hundred hand-labelled real ones. Train on the union as-is and the real
frames are under 5% of the gradient: the model learns sim and politely
ignores reality. --balance repeats the smaller domain so each contributes a
chosen share. Repetition is not as good as more real data, but it is what
turns "ignored" into "learned".

SEPARATE VALIDATION. A single merged val number is worthless here: 95% could
be 97% sim and 40% real, and you would ship it. This writes three configs --
data.yaml to train on, and data_val_sim.yaml / data_val_real.yaml to score
each domain on its own. Always report both.

    python3 mix_datasets.py --sim ~/esda_datasets/real_sim \\
                            --real ~/esda_datasets/real_photos \\
                            --out ~/esda_datasets/mixed --balance 0.35
    python3 train.py --data ~/esda_datasets/mixed/data.yaml
    # then, after training:
    yolo segment val model=best.pt data=~/esda_datasets/mixed/data_val_real.yaml
"""
import argparse
import os
import shutil

YAML = ("path: {path}\ntrain: {train}\nval: {val}\n\n"
        "names:\n  0: lane\n  1: cone\n  2: drivable\n")


def link_split(src, dst, split, tag, repeat=1):
    """Hard-link one split across, prefixing names so domains stay separable."""
    si = os.path.join(src, "images", split)
    sl = os.path.join(src, "labels", split)
    if not os.path.isdir(si):
        return 0
    di = os.path.join(dst, "images", split)
    dl = os.path.join(dst, "labels", split)
    os.makedirs(di, exist_ok=True)
    os.makedirs(dl, exist_ok=True)
    n = 0
    for f in sorted(os.listdir(si)):
        if not f.lower().endswith((".jpg", ".png", ".jpeg")):
            continue
        stem, ext = os.path.splitext(f)
        lab = os.path.join(sl, stem + ".txt")
        if not os.path.exists(lab):
            continue
        for r in range(repeat):
            suf = "" if r == 0 else f"_r{r}"
            name = f"{tag}_{stem}{suf}"
            for s, d in ((os.path.join(si, f), os.path.join(di, name + ext)),
                         (lab, os.path.join(dl, name + ".txt"))):
                if os.path.exists(d):
                    continue
                try:
                    os.link(s, d)          # cheap: no second copy on disk
                except OSError:
                    shutil.copy2(s, d)
            n += 1
    return n


def count(ds, split):
    d = os.path.join(ds, "images", split)
    return len([f for f in os.listdir(d)]) if os.path.isdir(d) else 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", required=True, help="autolabel.py output")
    p.add_argument("--real", default="", help="hand-labelled real photos")
    p.add_argument("--out", required=True)
    p.add_argument("--balance", type=float, default=0.35,
                   help="target share of TRAIN frames that are real, by "
                        "repetition. 0 disables, just concatenates")
    p.add_argument("--max-repeat", type=int, default=12,
                   help="cap on repeats; past this you need more photos, "
                        "not more copies of the same ones")
    a = p.parse_args()

    n_sim = count(a.sim, "train")
    n_real = count(a.real, "train") if a.real else 0
    rep = 1
    if a.real and a.balance > 0 and n_real:
        want = a.balance * n_sim / max(1e-9, 1.0 - a.balance)
        rep = max(1, min(a.max_repeat, int(round(want / n_real))))

    os.makedirs(a.out, exist_ok=True)
    tr = link_split(a.sim, a.out, "train", "sim")
    va = link_split(a.sim, a.out, "val", "sim")
    rtr = rrv = 0
    if a.real:
        rtr = link_split(a.real, a.out, "train", "real", repeat=rep)
        rrv = link_split(a.real, a.out, "val", "real")
        # a per-domain val view, so the two numbers can never be conflated
        for tag, src in (("sim", a.sim), ("real", a.real)):
            d = os.path.join(a.out, f"val_{tag}")
            os.makedirs(os.path.join(d, "images"), exist_ok=True)
            os.makedirs(os.path.join(d, "labels"), exist_ok=True)
            link_split(src, d, "val", tag)
            for sub in ("images", "labels"):
                s = os.path.join(d, sub, "val")
                if os.path.isdir(s):
                    for f in os.listdir(s):
                        os.replace(os.path.join(s, f), os.path.join(d, sub, f))
                    os.rmdir(s)
            with open(os.path.join(a.out, f"data_val_{tag}.yaml"), "w") as fh:
                fh.write(YAML.format(path=os.path.abspath(a.out),
                                     train="images/train",
                                     val=f"val_{tag}/images"))
    with open(os.path.join(a.out, "data.yaml"), "w") as fh:
        fh.write(YAML.format(path=os.path.abspath(a.out),
                             train="images/train", val="images/val"))

    share = rtr / max(1, tr + rtr)
    print(f"[mix] sim   train {tr:6d}  val {va:5d}")
    print(f"[mix] real  train {rtr:6d}  val {rrv:5d}   (x{rep} repeats)")
    print(f"[mix] real is {share*100:.0f}% of training frames")
    if a.real and rep >= a.max_repeat:
        print(f"[mix] NOTE repeats hit the cap of {a.max_repeat}. The model "
              "is seeing the same few photos over and over; take more rather "
              "than raising it.")
    if not a.real:
        print("[mix] no real data yet -- see vision/REAL_DATA.md")
    print(f"[mix] train on   {os.path.join(a.out, 'data.yaml')}")
    if a.real:
        print(f"[mix] then score BOTH data_val_sim.yaml and data_val_real.yaml")


if __name__ == "__main__":
    main()
