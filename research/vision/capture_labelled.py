#!/usr/bin/env python3
"""
capture_labelled.py -- save camera frames WITH the pose they were taken from.

capture_frames.py already saves jpgs, and that is where the old pipeline
stopped: a folder of pictures somebody then has to label by hand. The only
thing standing between those frames and a finished dataset is knowing where
the camera was, so this writes the pose alongside every frame and
autolabel.py computes the masks from the world's own geometry.

    frame_00042.jpg
    frame_00042.json   {"x":..., "y":..., "yaw":..., "world":..., "t":...}

POSE SOURCE. /odom, offset by the spawn pose. slam_toolbox's map frame is not
used on purpose: it jumps when the graph optimises, and a jump of 20 cm puts
every label 20 cm off for the frames around it. Wheel odometry in sim on flat
ground has no slip and drifts very little over a few minutes, which is what a
capture run is. For the generated arenas the robot spawns at the origin, so
odom and world coincide and --spawn can be left alone.

    python3 capture_labelled.py --world igvc_oval_test --out ~/esda_capture
    python3 capture_labelled.py --world igvc_oval --spawn 0.45,0.03,0.0
"""
import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_spawns

try:
    import cv2
    HAVE_CV = True
except ImportError:
    HAVE_CV = False


def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> HxWx3 uint8 BGR, without cv_bridge.

    Third copy of this in the repo, and deliberately so: cv_bridge is a C
    extension built against NumPy 1.x, the CUDA torch install brings NumPy
    2.x, and the first conversion dies with "_ARRAY_API not found". Every
    script that touches images carries its own so none of them can be broken
    by importing another.
    """
    enc = (msg.encoding or "").lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if enc == "rgb8" else img.copy()
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
        return img[:, :, ::-1].copy() if enc == "rgba8" else img.copy()
    if enc == "mono8":
        g = buf.reshape(msg.height, msg.width)
        return np.repeat(g[:, :, None], 3, axis=2)
    raise ValueError(f"unhandled image encoding {msg.encoding!r}")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Capture(Node):
    def __init__(self, a):
        super().__init__("capture_labelled")
        self.a = a
        self.pose = None
        self.n = 0
        self.last = 0.0
        self.skipped_no_pose = 0
        os.makedirs(a.out, exist_ok=True)
        self.create_subscription(Image, a.image_topic, self._on_img,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, a.odom_topic, self._on_odom, 20)
        self.create_timer(10.0, self._progress)
        self.get_logger().info(
            f"capturing {a.image_topic} every {a.interval}s -> {a.out} "
            f"(world {a.world}, spawn {a.spawn})")

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_img(self, msg):
        now = time.time()
        if now - self.last < self.a.interval:
            return
        if self.pose is None:
            self.skipped_no_pose += 1
            return
        self.last = now
        try:
            img = imgmsg_to_bgr(msg)
        except ValueError as e:                         # noqa: BLE001
            self.get_logger().warn(str(e))
            return
        # odom -> world: the spawn pose is the offset between the two.
        sx, sy, syaw = self.a._spawn
        ox, oy, oyaw = self.pose
        c, s = math.cos(syaw), math.sin(syaw)
        x = sx + c * ox - s * oy
        y = sy + s * ox + c * oy
        stem = f"frame_{self.n:05d}"
        cv2.imwrite(os.path.join(self.a.out, stem + ".jpg"), img)
        with open(os.path.join(self.a.out, stem + ".json"), "w") as f:
            json.dump({"x": x, "y": y, "yaw": oyaw + syaw,
                       "world": self.a.world, "t": now,
                       "odom": [ox, oy, oyaw]}, f)
        self.n += 1

    def _progress(self):
        self.get_logger().info(
            f"{self.n} frames saved"
            + (f", {self.skipped_no_pose} dropped waiting for odometry"
               if self.skipped_no_pose else ""))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", required=True)
    p.add_argument("--out", default="")
    p.add_argument("--image-topic", default="/head_camera/images")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--interval", type=float, default=0.5)
    p.add_argument("--spawn", default="",
                   help="x,y,yaw the sim spawned at. Default: looked up in "
                        "config/world_spawns.txt by --world.")
    p.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    a = p.parse_args()
    if not HAVE_CV:
        raise SystemExit("needs opencv: sudo apt install -y python3-opencv")
    # THE SPAWN IS NOT OPTIONAL AND IT IS NOT ZERO.
    #
    # It used to default to 0,0,0, which silently produced a whole capture of
    # wrong poses: job 109 recorded 672 frames of igvc_oval_slow_test whose
    # every pose was off by (11.86, -2.31, 1.263 rad), because nobody passed
    # --spawn and nothing complained. Labels drawn from those poses would have
    # landed in the wrong part of the arena and scored as a broken model.
    # So: look it up, and refuse to guess.
    listed = world_spawns.spawn_of(a.world)
    if a.spawn:
        a._spawn = tuple(float(v) for v in (a.spawn.split(",") + ["0", "0", "0"])[:3])
        if listed and max(abs(x - y) for x, y in zip(a._spawn, listed)) > 1e-6:
            print(f"[capture] NOTE --spawn {a._spawn} overrides the table's "
                  f"{listed} for {a.world}")
    elif listed is not None:
        a._spawn = listed
        print(f"[capture] spawn {a._spawn} for {a.world} "
              f"(from {world_spawns.REL})")
    elif world_spawns.table():
        raise SystemExit(
            f"'{a.world}' is not in {world_spawns.REL} and no --spawn was "
            f"given.\nIf it really starts at the origin, pass --spawn 0,0,0 "
            f"to say so on purpose. Listed worlds: "
            + ", ".join(sorted(world_spawns.table())))
    else:
        raise SystemExit(f"no spawn table at {world_spawns.table_path()} "
                         f"-- pass --spawn x,y,yaw explicitly")
    a.out = a.out or os.path.expanduser(f"~/esda_capture/{a.world}")

    rclpy.init()
    n = Capture(a)
    t_end = time.time() + a.seconds if a.seconds else None
    try:
        while rclpy.ok() and (t_end is None or time.time() < t_end):
            rclpy.spin_once(n, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.get_logger().info(f"saved {n.n} frames to {a.out}")
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
