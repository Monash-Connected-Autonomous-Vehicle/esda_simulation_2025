#!/usr/bin/env python3
"""record_video.py -- write an image topic straight to a video file.

demo.sh record already shells out to image_view's video_recorder, which needs
ros-humble-image-view installed. This needs only cv2, which the vision node
already depends on, so a recording cannot fail for want of a package on the
one occasion somebody wanted to show the thing working.

Writes mp4 when the build has the encoder and falls back to MJPG .avi when it
does not, because a large playable file beats a small missing one.

    python3 vision/record_video.py --topic /lane/debug_image \\
            --out ~/esda_runs/demo.mp4 --seconds 600 --fps 20
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import cv2
    HAVE_CV = True
except ImportError:
    HAVE_CV = False


def open_writer(path, w, h, fps):
    """A writer and the path it actually wrote to. mp4 first, then avi.

    cv2.VideoWriter does NOT raise when a codec is missing -- isOpened()
    returns False and every write is silently dropped, leaving a 0-byte file
    and no error. So check, and fall back.
    """
    attempts = [(path, "mp4v")] if path.endswith(".mp4") else []
    attempts.append((os.path.splitext(path)[0] + ".avi", "MJPG"))
    for p, fourcc in attempts:
        vw = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*fourcc), fps, (w, h))
        if vw.isOpened():
            return vw, p
        vw.release()
    raise SystemExit(f"no usable video encoder for {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--topic", default="/lane/debug_image")
    p.add_argument("--out", default=os.path.expanduser("~/esda_runs/run.mp4"))
    p.add_argument("--seconds", type=float, default=600.0, help="wall clock")
    p.add_argument("--fps", type=float, default=20.0, help="playback rate")
    a = p.parse_args()
    if not HAVE_CV:
        raise SystemExit("needs opencv")

    if a.selftest:
        import tempfile
        d = tempfile.mkdtemp()
        vw, path = open_writer(os.path.join(d, "t.mp4"), 64, 48, 10)
        for i in range(10):
            vw.write(np.full((48, 64, 3), i * 20, np.uint8))
        vw.release()
        ok = os.path.exists(path) and os.path.getsize(path) > 0
        print("wrote %s, %d bytes" % (os.path.basename(path),
                                      os.path.getsize(path) if ok else 0))
        print("ALL CHECKS PASSED" if ok else "FAILED: empty video")
        return 0 if ok else 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

    class Rec(Node):
        def __init__(self):
            super().__init__("record_video")
            self.vw = None
            self.path = None
            self.n = 0
            self.create_subscription(Image, a.topic, self._img,
                                     qos_profile_sensor_data)
            self.get_logger().info(f"recording {a.topic} -> {a.out}")

        def _img(self, m):
            buf = np.frombuffer(m.data, np.uint8)
            if m.encoding not in ("rgb8", "bgr8"):
                return
            img = buf.reshape(m.height, m.width, 3)
            if m.encoding == "rgb8":
                img = img[:, :, ::-1]
            if self.vw is None:
                self.vw, self.path = open_writer(a.out, m.width, m.height, a.fps)
                self.get_logger().info(f"writing {m.width}x{m.height} -> {self.path}")
            self.vw.write(np.ascontiguousarray(img))
            self.n += 1

    rclpy.init()
    n = Rec()
    t_end = time.time() + a.seconds
    try:
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if n.vw is not None:
            n.vw.release()
        size = os.path.getsize(n.path) if n.path and os.path.exists(n.path) else 0
        print(f"[rec] {n.n} frames -> {n.path} ({size/1e6:.1f} MB)")
        n.destroy_node()
        rclpy.shutdown()
    return 0 if n.n else 1


if __name__ == "__main__":
    raise SystemExit(main())
