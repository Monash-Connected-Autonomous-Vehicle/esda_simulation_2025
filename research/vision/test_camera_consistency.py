#!/usr/bin/env python3
"""The camera geometry lives in three places. They must agree.

  1. esda_simulation_2025/description/camera.xacro  -- the sensor Gazebo
     renders, and the mount it renders from
  2. vision/vision_config.yaml [projection]         -- what _project() uses
     to turn a mask pixel into a ground point
  3. vision/synth_gen.py                            -- the synthetic TRAINING
     images, and the ground truth autolabel.py computes

Nothing checks them against each other at runtime, and _project() uses the
config rather than TF, so a disagreement does not crash: it silently places
every lane point somewhere the paint is not.

This has now happened three times in one project:

  * fx/fy vs the sensor's horizontal_fov
  * cam_height vs the xacro joint, after the camera was raised
  * synth_gen left at the ORIGINAL rig (fx 528.43, 0.4775 m, level) for a
    day after the camera became 90 deg, 1.1 m and 20 deg down -- which would
    have trained the model for a camera that does not exist and then scored
    it against labels for a camera that does not exist

synth_gen now derives from the config, so (2) and (3) cannot drift. This
test exists for (1), which cannot be derived because it is the physical
truth the other two are describing.
"""
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.join(HERE, "..", "esda_simulation_2025", "description")

FAILS = []
RUN = 0


def check(name, got, want, tol=1e-2, unit=""):
    global RUN
    RUN += 1
    ok = abs(float(got) - float(want)) <= tol
    print("  %-30s xacro %12.4f   config %12.4f %-3s %s"
          % (name, want, got, unit, "ok" if ok else "MISMATCH"))
    if not ok:
        FAILS.append(name)


def num(pattern, text, what):
    m = re.search(pattern, text)
    if not m:
        raise SystemExit("could not read %s from the xacro" % what)
    return float(m.group(1))


cam = open(os.path.join(DESC, "camera.xacro")).read()
core = open(os.path.join(DESC, "robot_core_ref.xacro")).read()

# --- the physical truth, read from the model ---
wheel_r = num(r'name="wheel_radius"\s+value="([\d.]+)"', core, "wheel_radius")
off_x = num(r'name="wheel_offset_x"\s+value="([\d.]+)"', core, "wheel_offset_x")
off_z = num(r'name="wheel_offset_z"\s+value="([\d.]+)"', core, "wheel_offset_z")
jx, jz = re.search(r'<origin xyz="([\d.-]+) 0 ([\d.-]+)" rpy="0 ([\d.-]+) 0"/>',
                   cam).group(1, 2)
jpitch = re.search(r'<origin xyz="[\d.-]+ 0 [\d.-]+" rpy="0 ([\d.-]+) 0"/>',
                   cam).group(1)
jx, jz, jpitch = float(jx), float(jz), float(jpitch)
hfov = num(r"<horizontal_fov>([\d.]+)</horizontal_fov>", cam, "horizontal_fov")
iw = num(r"<width>(\d+)</width>", cam, "width")
ih = num(r"<height>(\d+)</height>", cam, "height")

truth = {
    # base_link rides on the axle, chassis shares its height
    "cam_height": (wheel_r - off_z) + jz,
    "cam_x_offset": -off_x + jx,
    "cam_pitch_deg": math.degrees(jpitch),
    "fx": (iw / 2.0) / math.tan(hfov / 2.0),
    "fy": (iw / 2.0) / math.tan(hfov / 2.0),
    "cx": iw / 2.0,
    "cy": ih / 2.0,
}

print(__doc__.split("\n")[0])
print("\nsensor: %dx%d, hfov %.6f rad (%.2f deg), mount z %.4f, pitch %.6f rad"
      % (iw, ih, hfov, math.degrees(hfov), jz, jpitch))
print("chain : wheel_radius %.4f - wheel_offset_z %.4f + joint z %.4f = %.4f m\n"
      % (wheel_r, off_z, jz, truth["cam_height"]))

print("1. vision_config.yaml [projection] vs the xacro")
cfg = yaml.safe_load(open(os.path.join(HERE, "vision_config.yaml")))["projection"]
for k, v in truth.items():
    check(k, cfg[k], v, unit="m" if "height" in k or "offset" in k else "")

print("\n1b. every xacro and world is well-formed XML")
# THIS HAS NOW BROKEN THE SIM TWICE. XML forbids "--" inside a comment, and a
# xacro that will not parse means no robot_description, which means Gazebo
# never publishes /clock and the bring-up times out 90 seconds later with an
# error that says nothing about a comment. Both times it was a dash in prose
# ("in hand -- a real lens..."), which is invisible unless you are looking for
# it. So look for it, every job, before anything else runs.
import glob as _glob, re as _re, xml.etree.ElementTree as _ET
_files = sorted(_glob.glob(os.path.join(os.path.dirname(HERE), "esda_simulation_2025", "description", "*.xacro"))
                + _glob.glob(os.path.join(os.path.dirname(HERE), "esda_simulation_2025", "worlds", "*.sdf")))
_bad = []
for _f in _files:
    try:
        _ET.parse(_f)
    except Exception as _e:
        _bad.append((os.path.basename(_f), str(_e)))
        continue
    for _m in _re.finditer(r"<!--(.*?)-->", open(_f).read(), _re.S):
        if "--" in _m.group(1):
            _bad.append((os.path.basename(_f), "'--' inside a comment"))
RUN += 1
print("  %-30s %d files, %d problems   %s"
      % ("xml well-formed", len(_files), len(_bad), "ok" if not _bad else "BROKEN"))
for _n, _e in _bad:
    print("      %s: %s" % (_n, _e))
if _bad:
    FAILS.append("malformed xml")

print("\n2. synth_gen.py vs the xacro  (labels + training images)")
sys.path.insert(0, HERE)
import synth_gen as sg
for k, attr in (("cam_height", "CAM_HEIGHT"), ("cam_x_offset", "CAM_X_OFFSET"),
                ("cam_pitch_deg", "CAM_PITCH_DEG"), ("fx", "FX"),
                ("fy", "FY"), ("cx", "CX"), ("cy", "CY")):
    check(k, getattr(sg, attr), truth[k])
check("image width", sg.W, iw)
check("image height", sg.H_IMG, ih)

print("\n3. the projection actually round-trips at this geometry")
import numpy as np
X, Y, valid = sg.pixel_to_ground()
# Sample points DERIVED from the image size, not written down. The fixed
# rows 300/380/420 were for a 480-row frame and indexed off the end the
# moment the camera became 640x360 -- a test that breaks when the thing it
# guards changes legitimately is a test nobody will keep.
rows = np.nonzero(valid.any(axis=1))[0]
lo, hi = int(rows[0]), int(rows[-1])
vs = np.unique(np.clip(
    (lo + (hi - lo) * np.array([0.35, 0.60, 0.85])).astype(int), lo, hi))
us = np.clip((sg.W * np.array([0.25, 0.50, 0.75])).astype(int), 0, sg.W - 1)[:len(vs)]
ok = valid[vs, us]
u2, v2 = sg.ground_to_pixel(X[vs, us], Y[vs, us])
err = float(np.abs(u2[ok] - us[ok]).max() + np.abs(v2[ok] - vs[ok]).max()) if ok.any() else 99
RUN += 1
print("  %-30s max round-trip error %.4f px   %s"
      % ("pixel -> ground -> pixel", err, "ok" if err < 0.01 else "MISMATCH"))
if err >= 0.01:
    FAILS.append("round trip")

EXPECTED = 18
print("\n%d checks run (expected at least %d)" % (RUN, EXPECTED))
if RUN < EXPECTED:
    FAILS.append("only %d checks ran, expected %d+" % (RUN, EXPECTED))
print()
if FAILS:
    print("FAILURES -- the camera geometry has drifted apart:")
    for f in FAILS:
        print("  -", f)
    raise SystemExit(1)
print("ALL CHECKS PASSED -- xacro, vision_config.yaml and synth_gen.py agree")
