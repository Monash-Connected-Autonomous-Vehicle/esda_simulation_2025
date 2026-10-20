#!/usr/bin/env python3
"""
Generate extra Gazebo arenas so the model sees more than one course.

WHY
    Everything so far trains on one layout. The synthetic renderer varies lane
    geometry, but it is my compositor, not ogre2 -- so it cannot teach the
    model what real Gazebo shading, shadows and barrel geometry look like.
    These worlds close that: they are real Gazebo arenas, rendered by ogre2,
    with courses the model has never seen. Drive them, capture frames, and you
    get genuine in-domain variety instead of more of the same lap.

HOW IT WORKS (and why it costs almost nothing on disk)
    The arena floor is a single 43 x 37 m quad, textured by one PNG. Verified
    from meshes/ground.dae:
        verts  (-21.5,-18.5) (21.5,-18.5) (21.5,18.5) (-21.5,18.5), Z_UP
        UVs    (0,0) (1,0) (1,1) (0,1)
        png    1400 x 1200  ->  32.56 px/m in x, 32.43 px/m in y
    so world (X,Y) -> pixel ( (X+21.5)/43*1400 , (18.5-Y)/37*1200 ).

    A new course is therefore just a new PNG plus a 5 KB .dae pointing at it.
    Barrels are declared in the WORLD file and reference the EXISTING
    construction_barrel.dae by package:// URI, so nothing is duplicated --
    no new model directories, no copied meshes.

    The asphalt itself is the real ground.png with its painted lines
    inpainted out, so the surface is genuine sim material; only the course
    drawn on top is new.

USAGE
    python3 make_worlds.py --list
    python3 make_worlds.py --out ../esda_simulation_2025 --worlds oval,slalom,wavy,figure8
    python3 make_worlds.py --out ../esda_simulation_2025 --worlds all --preview

THEN
    ros2 launch esda_simulation_2025 launch_sim_headless.launch.py world:=igvc_oval
    ...or:  ./demo.sh up --world igvc_oval
"""
import argparse
import math
import os
import random
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import synth_gen as S

# ---- arena constants, read off ground.dae ---------------------------------
X_MIN, X_MAX = -21.5, 21.5
Y_MIN, Y_MAX = -18.5, 18.5
TEX_W, TEX_H = 1400, 1200
PX_PER_M_X = TEX_W / (X_MAX - X_MIN)
PX_PER_M_Y = TEX_H / (Y_MAX - Y_MIN)
LANE_BGR = (242, 242, 242)      # sampled from the real ground.png


def w2p(pts):
    """World (X,Y) metres -> texture pixel (col,row)."""
    p = np.asarray(pts, float).reshape(-1, 2)
    u = (p[:, 0] - X_MIN) / (X_MAX - X_MIN) * TEX_W
    v = (Y_MAX - p[:, 1]) / (Y_MAX - Y_MIN) * TEX_H
    return np.stack([u, v], 1)


# ---------------------------------------------------------------- courses --
# Each layout returns a LIST of segments: [(points Nx2, closed, width)].
# width None -> pick one at random. A list of segments lets a course be more
# than one corridor, which is what crossroads and fork need.

def smooth_path(pts, k=21, closed=False):
    """Moving-average smooth of a polyline.

    Offsetting a polyline laterally self-intersects wherever the turn is
    sharper than the offset distance -- that is what put a bowtie in the
    first hairpin and little boxes on every zigzag corner. Rounding the
    centreline first removes the cusps, and a real painted course has
    rounded corners anyway.
    """
    p = np.asarray(pts, float)
    if len(p) < k:
        return p
    pad = k // 2
    if closed:
        ext = np.vstack([p[-pad:], p, p[:pad]])
    else:
        ext = np.vstack([np.repeat(p[:1], pad, 0), p, np.repeat(p[-1:], pad, 0)])
    ker = np.ones(k) / k
    sx = np.convolve(ext[:, 0], ker, "same")[pad:-pad]
    sy = np.convolve(ext[:, 1], ker, "same")[pad:-pad]
    return np.stack([sx, sy], 1)


def _seg(pts, closed=True, width=None):
    return [(pts, closed, width)]


def course_oval(rng):
    a, b = rng.uniform(13, 17), rng.uniform(10, 13)
    t = np.linspace(0, 2 * np.pi, 400)
    n = rng.uniform(2.6, 4.0)                       # superellipse -> rounded box
    x = a * np.sign(np.cos(t)) * np.abs(np.cos(t)) ** (2 / n)
    y = b * np.sign(np.sin(t)) * np.abs(np.sin(t)) ** (2 / n)
    return _seg(np.stack([x, y], 1))


def course_wavy(rng):
    t = np.linspace(0, 2 * np.pi, 500)
    a, b = rng.uniform(13, 16), rng.uniform(9, 12)
    k = rng.choice([3, 4, 5])
    amp = rng.uniform(1.4, 2.8)
    x = (a + amp * np.sin(k * t)) * np.cos(t)
    y = (b + amp * np.cos(k * t)) * np.sin(t)
    return _seg(np.stack([x, y], 1))


def course_figure8(rng):
    t = np.linspace(0, 2 * np.pi, 600)
    a, b = rng.uniform(14, 17), rng.uniform(7, 10)
    return _seg(np.stack([a * np.sin(t), b * np.sin(t) * np.cos(t) * 2.0], 1))


def course_slalom(rng):
    y = np.linspace(-16.0, 16.0, 400)
    x = rng.uniform(1.5, 3.0) * np.sin(y / rng.uniform(4.0, 7.0)) + rng.uniform(-4, 4)
    return _seg(np.stack([x, y], 1), closed=False)


def course_spline(rng):
    k = rng.randint(5, 7)
    ang = np.sort(rng.sample(list(np.linspace(0, 2 * np.pi, 40)), k))
    rad = [rng.uniform(9.0, 16.0) for _ in range(k)]
    ang = np.append(ang, ang[0] + 2 * np.pi)
    rad.append(rad[0])
    t = np.linspace(0, 2 * np.pi, 600)
    r = np.interp(t, ang, rad, period=2 * np.pi)
    r = np.convolve(np.r_[r[-60:], r, r[:60]], np.ones(61) / 61, "same")[60:-60]
    return _seg(np.stack([r * np.cos(t), r * np.sin(t)], 1))


# ---- the hard ones --------------------------------------------------------

def course_hairpin(rng):
    """Stacked U-turns. Tight radii mean the corridor sweeps right across the
    camera instead of vanishing toward the horizon.

    Built as straight + exact semicircle with matched endpoints, then
    smoothed. The earlier version stitched arcs whose ends did not meet,
    which offset into a bowtie.
    """
    R = rng.uniform(3.2, 4.5)
    half = rng.uniform(4.5, 6.5)          # half-length of each straight
    n_turns = rng.randint(3, 4)
    pts = []
    y = -13.0
    going_right = True
    for i in range(n_turns):
        x0, x1 = (-half, half) if going_right else (half, -half)
        pts.append(np.stack([np.linspace(x0, x1, 90), np.full(90, y)], 1))
        if i < n_turns - 1:
            # semicircle centred at (x1, y+R), starting at the straight's end
            cx, cy = x1, y + R
            start = -np.pi / 2
            th = start + np.linspace(0, np.pi, 70) * (1 if going_right else -1)
            pts.append(np.stack([cx + R * np.cos(th) * (1 if going_right else -1),
                                 cy + R * np.sin(th)], 1))
            y += 2 * R
        going_right = not going_right
    path = np.vstack(pts)
    return _seg(smooth_path(path, 15), closed=False, width=rng.uniform(2.4, 3.2))


def course_crossroads(rng):
    """Two corridors crossing. Lines run diagonally across frame and meet at
    a sharp angle -- the geometry a single loop never produces."""
    ang = rng.uniform(0.6, 1.1)        # crossing angle, radians
    L = 16.0
    t = np.linspace(-L, L, 300)
    segs = []
    for a in (0.0, ang):
        d = np.array([math.cos(a), math.sin(a)])
        segs.append((np.outer(t, d), False, rng.uniform(2.6, 3.6)))
    return segs


def course_narrowing(rng):
    """Corridor that pinches from ~4.5 m down to ~1.8 m and opens again."""
    t = np.linspace(0, 2 * np.pi, 500)
    a, b = rng.uniform(13, 16), rng.uniform(9, 12)
    pts = np.stack([a * np.cos(t), b * np.sin(t)], 1)
    w = 3.2 + 1.3 * np.sin(rng.choice([2, 3]) * t + rng.uniform(0, 6.28))
    return [(pts, True, np.clip(w, 1.8, 4.6))]


def course_zigzag(rng):
    """Sawtooth. Sharp corners, so the lane leaves the frame edge often."""
    n = rng.randint(5, 8)
    amp = rng.uniform(5.0, 9.0)
    ys = np.linspace(-15.0, 15.0, n)
    xs = np.array([amp * (1 if i % 2 else -1) for i in range(n)]) + rng.uniform(-3, 3)
    out = []
    for i in range(n - 1):
        out.append(np.stack([np.linspace(xs[i], xs[i + 1], 60),
                             np.linspace(ys[i], ys[i + 1], 60)], 1))
    # smooth, or every corner offsets into a self-intersecting box
    return _seg(smooth_path(np.vstack(out), 25), closed=False,
                width=rng.uniform(2.4, 3.4))


def course_fork(rng):
    """A corridor that splits. Forces the model to handle three lines in
    frame at once rather than a tidy left/right pair."""
    stem = np.stack([np.zeros(120), np.linspace(-15.0, -2.0, 120)], 1)
    segs = [(stem, False, rng.uniform(2.8, 3.6))]
    for sgn in (-1, 1):
        a = rng.uniform(0.5, 0.9) * sgn
        t = np.linspace(0, 16.0, 160)
        branch = np.stack([t * math.sin(a), -2.0 + t * math.cos(a)], 1)
        segs.append((branch, False, rng.uniform(2.4, 3.2)))
    return segs


COURSES = {
    "oval": course_oval,
    "wavy": course_wavy,
    "figure8": course_figure8,
    "slalom": course_slalom,
    "spline": course_spline,
    # harder: tight radii, crossings, varying width, branches
    "hairpin": course_hairpin,
    "crossroads": course_crossroads,
    "narrowing": course_narrowing,
    "zigzag": course_zigzag,
    "fork": course_fork,
}
HARD = ("hairpin", "crossroads", "narrowing", "zigzag", "fork")


# ----------------------------------------------------------------- texture --
def offset_path(path, closed, d):
    """Offset a polyline laterally by d metres (left positive).

    d may be a scalar or a per-point array, which is how `narrowing` pinches
    the corridor along its length.
    """
    p = np.asarray(path, float)
    if closed:
        prev = np.roll(p, 1, 0)
        nxt = np.roll(p, -1, 0)
    else:
        prev = np.vstack([p[0], p[:-1]])
        nxt = np.vstack([p[1:], p[-1]])
    t = nxt - prev
    n = np.stack([-t[:, 1], t[:, 0]], 1)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 1e-9)
    d = np.asarray(d, float).reshape(-1, 1) if np.ndim(d) else d
    return p + n * d


def _stroke(pts_px, thick_px, closed):
    """Binary mask of a polyline stroked with a (possibly varying) width."""
    m = np.zeros((TEX_H, TEX_W), np.uint8)
    n = len(pts_px)
    rng_end = n if closed else n - 1
    tp = np.asarray(thick_px, float)
    for i in range(rng_end):
        j = (i + 1) % n
        t = tp if tp.ndim == 0 else tp[i]
        t = int(max(1, round(float(t))))
        cv2.line(m, tuple(pts_px[i]), tuple(pts_px[j]), 255, t, cv2.LINE_8)
    return m > 0


def draw_course(plate, segments, rng):
    """Paint every segment's two lane lines onto the asphalt plate.

    Uses a distance band rather than an offset polyline -- see band_draw.py's
    note. Returns (image, resolved_segments) with the width actually used, so
    barrel placement can straddle the same lines.
    """
    img = plate.copy()
    paint_m = rng.uniform(0.10, 0.16)
    resolved = []
    lane_any = np.zeros((TEX_H, TEX_W), bool)

    for pts_w, closed, width in segments:
        if width is None:
            width = rng.uniform(2.8, 4.2)
        resolved.append((pts_w, closed, width))   # original path, pre-extension

        # An open path stroked with a thickness gets END CAPS, and the band
        # then draws a line straight across the corridor mouth -- slalom came
        # out as a closed sausage. Run the ends well past the arena edge so
        # the caps fall outside the textured quad and never render.
        if not closed:
            pts_w = np.asarray(pts_w, float)
            d0 = pts_w[0] - pts_w[min(3, len(pts_w) - 1)]
            d1 = pts_w[-1] - pts_w[max(-4, -len(pts_w))]
            for d in (d0, d1):
                nrm = np.linalg.norm(d)
                if nrm > 1e-9:
                    d /= nrm
            pts_w = np.vstack([pts_w[0] + d0 * 8.0, pts_w, pts_w[-1] + d1 * 8.0])

        px = w2p(pts_w).astype(np.int32)
        w_arr = np.full(len(pts_w), float(width)) if np.ndim(width) == 0 \
            else np.asarray(width, float)
        outer_t = (w_arr + paint_m) * PX_PER_M_X
        inner_t = (w_arr - paint_m) * PX_PER_M_X

        band = _stroke(px, outer_t, closed)
        if float(np.min(inner_t)) > 1.0:
            band &= ~_stroke(px, inner_t, closed)

        if rng.random() < 0.35:                      # dashed: mask the band
            dash = np.zeros((TEX_H, TEX_W), bool)
            seg = max(2, len(px) // max(8, len(px) // 14))
            for i in range(0, len(px) - seg, seg * 2):
                dash |= _stroke(px[i:i + seg], outer_t if np.ndim(outer_t) == 0
                                else outer_t[i:i + seg], False)
            band &= dash
        lane_any |= band

    noise = np.random.randn(int(lane_any.sum()), 3) * 4
    img[lane_any] = np.clip(np.array(LANE_BGR, float) + noise, 0, 255).astype(np.uint8)

    for _ in range(rng.randint(2, 6)):
        a = np.array([rng.uniform(X_MIN + 2, X_MAX - 2),
                      rng.uniform(Y_MIN + 2, Y_MAX - 2)])
        b = a + np.array([rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)])
        q = w2p([a, b]).astype(np.int32)
        cv2.line(img, tuple(q[0]), tuple(q[1]), (30, 30, 30),
                 rng.randint(2, 5), cv2.LINE_AA)
    return img, resolved


# -------------------------------------------------------------------- dae --
DAE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images>
    <image id="ground_png" name="ground_png">
      <init_from>../materials/textures/{tex}</init_from>
    </image>
  </library_images>
  <library_effects>
    <effect id="ground_fx">
      <profile_COMMON>
        <newparam sid="ground_surf"><surface type="2D"><init_from>ground_png</init_from></surface></newparam>
        <newparam sid="ground_samp"><sampler2D><source>ground_surf</source></sampler2D></newparam>
        <technique sid="common">
          <lambert>
            <emission><color>0 0 0 1</color></emission>
            <diffuse><texture texture="ground_samp" texcoord="UVSET0"/></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="ground_mat" name="ground_mat"><instance_effect url="#ground_fx"/></material>
  </library_materials>
  <library_geometries>
    <geometry id="ground_geo" name="ground_geo">
      <mesh>
        <source id="ground_pos">
          <float_array id="ground_pos_array" count="12">-21.5 -18.5 0 21.5 -18.5 0 21.5 18.5 0 -21.5 18.5 0</float_array>
          <technique_common><accessor source="#ground_pos_array" count="4" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="ground_nrm">
          <float_array id="ground_nrm_array" count="12">0 0 1 0 0 1 0 0 1 0 0 1</float_array>
          <technique_common><accessor source="#ground_nrm_array" count="4" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
          </accessor></technique_common>
        </source>
        <source id="ground_uv">
          <float_array id="ground_uv_array" count="8">0 0 1 0 1 1 0 1</float_array>
          <technique_common><accessor source="#ground_uv_array" count="4" stride="2">
            <param name="S" type="float"/><param name="T" type="float"/>
          </accessor></technique_common>
        </source>
        <vertices id="ground_vtx"><input semantic="POSITION" source="#ground_pos"/></vertices>
        <triangles count="2" material="ground_mat">
          <input semantic="VERTEX" source="#ground_vtx" offset="0"/>
          <input semantic="NORMAL" source="#ground_nrm" offset="1"/>
          <input semantic="TEXCOORD" source="#ground_uv" offset="2" set="0"/>
          <p>0 0 0  1 1 1  2 2 2  2 2 2  3 3 3  0 0 0</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene" name="scene">
      <node id="ground" name="ground">
        <instance_geometry url="#ground_geo">
          <bind_material><technique_common>
            <instance_material symbol="ground_mat" target="#ground_mat">
              <bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>
            </instance_material>
          </technique_common></bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""


# ------------------------------------------------------------------ world --
def barrel_link(name, x, y, yaw):
    return f"""    <link name="{name}">
      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>package://models/orange_igvc/meshes/construction_barrel.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>package://models/orange_igvc/meshes/construction_barrel.dae</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
"""


# launch_sim_headless.launch.py spawns the robot with "-z 0.3" and no x/y,
# i.e. at the origin. A barrel there would spawn inside the robot and either
# launch it across the arena or wedge it. Keep a clear disc around (0,0).
SPAWN_CLEAR_M = 3.0


def place_barrels(segments, rng):
    """Barrels on and beside the painted lines -- the case that matters."""
    out = []
    for pts_w, closed, width in segments:
        n = len(pts_w)
        step = max(6, n // rng.randint(10, 20))
        half = width / 2 if np.ndim(width) == 0 else np.asarray(width) / 2
        for i in range(0, n, step):
            if rng.random() < 0.35:
                continue
            side = rng.choice([-1, 1])
            hw = half if np.ndim(half) == 0 else half[i]
            off = side * hw + rng.uniform(-0.35, 0.35)
            e = offset_path(pts_w, closed, side * half + rng.uniform(-0.35, 0.35)
                            if np.ndim(half) else off)[i]
            if X_MIN + 1.5 < e[0] < X_MAX - 1.5 and Y_MIN + 1.5 < e[1] < Y_MAX - 1.5:
                out.append((e[0], e[1], rng.uniform(0, 6.28)))
    for _ in range(rng.randint(3, 8)):
        out.append((rng.uniform(X_MIN + 3, X_MAX - 3),
                    rng.uniform(Y_MIN + 3, Y_MAX - 3), rng.uniform(0, 6.28)))
    keep = [b for b in out if math.hypot(b[0], b[1]) > SPAWN_CLEAR_M]
    if len(keep) != len(out):
        print(f"         (dropped {len(out) - len(keep)} barrel(s) within "
              f"{SPAWN_CLEAR_M:.0f} m of the robot spawn)")
    return keep


def ramp_links(segments, n_pairs, rng):
    """Shallow ramp plates across the corridor, so driving over one pitches
    and rolls the camera. Geometry copied from the stock world's ramp_part1/2:
    a 4 x 2 x 0.05 plate, roll 0.148 rad (~8.5 deg), centre z 0.1484, mirrored
    in pairs so the robot goes up one side and down the other.

    OFF by default. A ramp the robot cannot climb is worse than no ramp, and
    none of this has been driven in Gazebo.
    """
    if n_pairs <= 0:
        return "", 0
    pts_w, closed, _ = segments[0]
    n = len(pts_w)
    xml, placed = "", 0
    tried = 0
    idxs = [int(n * (k + 0.5) / n_pairs) % n for k in range(n_pairs)]
    for k, i in enumerate(idxs):
        tried += 1
        p = pts_w[i]
        if math.hypot(p[0], p[1]) < SPAWN_CLEAR_M + 2.0:
            continue
        if not (X_MIN + 4 < p[0] < X_MAX - 4 and Y_MIN + 4 < p[1] < Y_MAX - 4):
            continue
        nxt = pts_w[(i + 3) % n] if closed else pts_w[min(i + 3, n - 1)]
        th = math.atan2(nxt[1] - p[1], nxt[0] - p[0])      # path heading
        dx, dy = math.cos(th), math.sin(th)
        for sgn, roll in ((+1, -0.148), (-1, 0.148)):
            cx = p[0] + sgn * 0.9889 * dx
            cy = p[1] + sgn * 0.9889 * dy
            xml += f"""    <link name="ramp{k}_{'a' if sgn > 0 else 'b'}">
      <pose>{cx:.4f} {cy:.4f} 0.1484 {roll:.4f} 0 {th + math.pi / 2:.4f}</pose>
      <visual name="visual">
        <geometry><box><size>4 2 0.05</size></box></geometry>
        <material><script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Green</name>
        </script></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>4 2 0.05</size></box></geometry>
        <surface>
          <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
          <bounce/><contact><ode/></contact>
        </surface>
      </collision>
    </link>
"""
        placed += 1
    return xml, placed


def world_sdf(name, dae, barrels, rng, ramps_xml=""):
    az = rng.uniform(0, 2 * math.pi)
    el = rng.uniform(0.55, 0.95)
    dx, dy = math.cos(az) * (1 - el), math.sin(az) * (1 - el)
    diff = rng.uniform(0.6, 0.95)
    links = "".join(barrel_link(f"barrel{i}", x, y, yaw)
                    for i, (x, y, yaw) in enumerate(barrels)) + ramps_xml
    return f"""<?xml version="1.0" ?>
<!-- Generated by vision/make_worlds.py. Layout, barrel field and sun are
     randomised; the asphalt is the real ground.png with its original painted
     lines inpainted out, so the surface material matches the stock arena. -->
<sdf version="1.8">
  <world name="{name}">
    <physics name="1ms" type="sdformat">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"></plugin>
    <plugin filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"></plugin>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
    <plugin filename="libignition-gazebo-sensors-system.so"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>{diff:.2f} {diff:.2f} {diff:.2f} 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>{dx:.3f} {dy:.3f} -{el:.3f}</direction>
    </light>
    <model name="arena">
      <static>true</static>
      <link name="ground">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>package://models/orange_igvc/meshes/{dae}</uri>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry><plane><size>43 37</size></plane></geometry>
          <surface>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
            <bounce/><contact><ode/></contact>
          </surface>
        </collision>
      </link>
{links}    </model>
  </world>
</sdf>
"""


# ------------------------------------------------------------------- main --
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--out", default=os.path.join(here, "..", "esda_simulation_2025"),
                    help="the esda_simulation_2025 package root")
    ap.add_argument("--worlds", default="oval,slalom,wavy,figure8",
                    help="comma list, 'all', or 'hard' for the difficult ones")
    ap.add_argument("--ramps", type=int, default=0,
                    help="ramp pairs per world, for real camera pitch/roll. "
                         "OFF by default -- untested in Gazebo, and a ramp the "
                         "robot cannot climb is worse than none.")
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--tag", default="",
                    help="suffix for the generated names, e.g. --tag test "
                         "makes igvc_oval_test. Use this for a HELD-OUT set: "
                         "the ten arenas shipped with the repo were built with "
                         "--seed 11, so measuring on them measures the course "
                         "the parameters were tuned against. A different seed "
                         "AND a different name keeps the two sets apart.")
    ap.add_argument("--preview", action="store_true",
                    help="also write a downscaled PNG of each course next to it")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    if a.list:
        print("available course layouts:")
        for k in COURSES:
            print(f"  {k}")
        return

    if a.worlds == "all":
        names = list(COURSES)
    elif a.worlds == "hard":
        names = list(HARD)
    else:
        names = [w.strip() for w in a.worlds.split(",")]
    bad = [n for n in names if n not in COURSES]
    if bad:
        sys.exit(f"unknown layout(s): {bad}. Try --list")

    pkg = os.path.abspath(os.path.expanduser(a.out))
    tex_dir = os.path.join(pkg, "models", "orange_igvc", "materials", "textures")
    mesh_dir = os.path.join(pkg, "models", "orange_igvc", "meshes")
    world_dir = os.path.join(pkg, "worlds")
    for d in (tex_dir, mesh_dir, world_dir):
        if not os.path.isdir(d):
            sys.exit(f"not found: {d}\nPoint --out at the esda_simulation_2025 package root.")

    src_png = os.path.join(tex_dir, "ground.png")
    if not os.path.isfile(src_png):
        sys.exit(f"missing {src_png}")
    print("[worlds] building the lane-free asphalt plate from the real ground.png")
    plate = S.build_asphalt_plate(src_png, cache="/tmp/esda_asphalt_arena.png")
    plate = cv2.resize(plate, (TEX_W, TEX_H))

    for i, nm in enumerate(names):
        rng = random.Random(a.seed + i * 7919)
        segments = COURSES[nm](rng)
        base = plate.copy()
        if rng.random() < 0.5:
            base = cv2.flip(base, rng.choice([0, 1]))        # vary the asphalt
        img, segments = draw_course(base, segments, rng)

        stem = nm + (("_" + a.tag) if a.tag else "")
        tex = f"ground_{stem}.png"
        dae = f"ground_{stem}.dae"
        world = f"igvc_{stem}"

        cv2.imwrite(os.path.join(tex_dir, tex), img,
                    [cv2.IMWRITE_PNG_COMPRESSION, 9])
        with open(os.path.join(mesh_dir, dae), "w") as f:
            f.write(DAE.format(tex=tex))
        barrels = place_barrels(segments, rng)
        ramps_xml, n_ramps = ramp_links(segments, a.ramps, rng)
        with open(os.path.join(world_dir, world + ".sdf"), "w") as f:
            f.write(world_sdf(world, dae, barrels, rng, ramps_xml))

        sz = os.path.getsize(os.path.join(tex_dir, tex)) / 1e6
        ws = [w for _, _, w in segments]
        wtxt = (f"{np.min([np.min(w) for w in ws]):.1f}-"
                f"{np.max([np.max(w) for w in ws]):.1f} m")
        print(f"[worlds] {world:<16} {len(segments)} seg | corridor {wtxt:<10} "
              f"| {len(barrels):3d} barrels | {n_ramps} ramp pairs "
              f"| texture {sz:.1f} MB")

        if a.preview:
            prev = cv2.resize(img, (560, 480))
            cv2.imwrite(os.path.join(tex_dir, f"preview_{nm}.png"), prev)

    print()
    print("Launch one with:")
    print(f"    ros2 launch esda_simulation_2025 launch_sim_headless.launch.py world:=igvc_{names[0]}")
    print("or: ./demo.sh up --world igvc_" + names[0])
    print()
    print("Rebuild first so the new files are installed:  bash setup_wsl.sh --sync")


if __name__ == "__main__":
    main()
