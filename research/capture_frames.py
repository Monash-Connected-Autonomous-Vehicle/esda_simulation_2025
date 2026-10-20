#!/usr/bin/env python3
"""
Capture camera frames from the sim (or real camera) for building a YOLO lane dataset.

Subscribes to an image topic and saves a frame every N seconds to a folder, ready
to upload to Roboflow for labelling.

Run (ROS 2 sourced):
    python3 capture_frames.py
    python3 capture_frames.py /head_camera/images 0.5 ~/lane_dataset   # topic, interval(s), out dir

Default: /head_camera/images, 1 frame/sec, saved to ~/lane_dataset/
Stop with Ctrl+C. Then upload ~/lane_dataset/*.jpg to a Roboflow project.

Needs: ros-humble-cv-bridge + python3-opencv
    sudo apt install -y ros-humble-cv-bridge python3-opencv
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import numpy as np

try:
    import cv2
    HAVE_CV = True
except Exception:
    HAVE_CV = False


def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> HxWx3 uint8 BGR, without cv_bridge.

    Same reason as vision/lane_seg_node.py: cv_bridge is a C extension built
    against NumPy 1.x, a CUDA torch install brings NumPy 2.x, and the first
    conversion then dies with "AttributeError: _ARRAY_API not found". Kept as
    a copy rather than a shared import because this script lives at the repo
    root and must stay runnable on its own.
    """
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    nch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if nch is None:
        raise ValueError("unsupported image encoding: %s" % msg.encoding)
    row = msg.step if msg.step else msg.width * nch      # step is BYTES/row
    a = buf[:msg.height * row].reshape(msg.height, row)
    a = a[:, :msg.width * nch].reshape(msg.height, msg.width, nch)
    if enc == "mono8":
        return np.repeat(a, 3, axis=2)
    a = a[:, :, :3]
    if enc.startswith("rgb"):
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a)


class FrameCapture(Node):
    def __init__(self, topic, interval, out_dir):
        super().__init__("frame_capture")
        self.interval = float(interval)
        self.out_dir = os.path.expanduser(out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.last = 0.0
        self.n = 0
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"capturing '{topic}' -> {self.out_dir}  (1 frame / {self.interval:.2f}s)")
        if not HAVE_CV:
            self.get_logger().warn("cv2 missing — using raw PPM fallback. "
                                   "Install: sudo apt install -y python3-opencv")

    def cb(self, msg):
        now = time.time()
        if now - self.last < self.interval:
            return
        self.last = now
        ts = time.strftime("%Y%m%d_%H%M%S")
        name = f"frame_{ts}_{self.n:04d}"
        try:
            if HAVE_CV:
                img = imgmsg_to_bgr(msg)
                path = os.path.join(self.out_dir, name + ".jpg")
                cv2.imwrite(path, img)
            else:
                # minimal fallback: write a PPM (no cv2 needed)
                arr = imgmsg_to_bgr(msg)[:, :, ::-1]   # PPM wants RGB
                path = os.path.join(self.out_dir, name + ".ppm")
                with open(path, "wb") as f:
                    f.write(f"P6\n{msg.width} {msg.height}\n255\n".encode())
                    f.write(arr[:, :, :3].tobytes())
            self.n += 1
            if self.n % 10 == 0:
                self.get_logger().info(f"saved {self.n} frames (latest: {os.path.basename(path)})")
        except Exception as e:
            self.get_logger().error(f"save failed: {e}")


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/head_camera/images"
    interval = sys.argv[2] if len(sys.argv) > 2 else "1.0"
    out_dir = sys.argv[3] if len(sys.argv) > 3 else "~/lane_dataset"
    rclpy.init()
    node = FrameCapture(topic, interval, out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"done — {node.n} frames in {node.out_dir}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
