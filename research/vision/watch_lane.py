#!/usr/bin/env python3
"""Save what the lane detector SAW, tagged with where the robot was.

WHY THIS EXISTS

The camera starvation is fixed: the vision node now publishes 1516 times in
a run instead of 42, at the full 5 Hz the sensor is asked for, with 20 ms of
inference. The robot drove 28.7 m of clean arc at 0.38 m/s with the carrot
out at the full 2.75 m lookahead -- and then, at roughly map (7.5, 15),
stopped finding paint and never found it again:

    carrot (2.75,+0.00) base_link -> (7.53,14.91) map  [3 band(s), ...]
    carrot (2.75,+0.00) base_link -> (7.37,16.23) map  [blind corridor]
    not steering: no sensing for 8s  ... 18s ... 994s

Two explanations fit that equally well from the logs alone, and they lead
opposite ways:

  (a) there IS paint in frame and the model misses it      -> train the model
  (b) there is NO paint in frame, because the robot has    -> fix the geometry,
      cut the corner or is aimed at blank ground              not the weights

Last time this project guessed between those two it guessed (a) and was
wrong -- the robot was in the oval's infield with nothing to see. So this
time: look at the pictures.

/lane/debug_image already carries the model's own masks and boxes drawn on
the frame it ran on, so one subscription answers it. Each file is named with
the elapsed time and the robot's map position, which is what makes a frame
findable from a drive log afterwards.

    python3 watch_lane.py --out ~/lane_watch --interval 2.0 --seconds 400
"""
import argparse
import csv
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry


def imgmsg_to_bgr(msg):
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img if msg.encoding == "bgr8" else img[:, :, ::-1].copy()
    if msg.encoding == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError("unhandled encoding %s" % msg.encoding)


class Watcher(Node):
    def __init__(self, a):
        super().__init__("watch_lane")
        self.a = a
        os.makedirs(a.out, exist_ok=True)
        self.x = self.y = 0.0
        self.have_pose = False
        self.n_pts = 0
        self.saved = 0
        self.t0 = time.monotonic()
        self._last = 0.0
        self.csv = open(os.path.join(a.out, "index.csv"), "w", newline="")
        self.w = csv.writer(self.csv)
        self.w.writerow(["file", "t_wall_s", "x_odom", "y_odom", "lane_points"])
        self.create_subscription(Odometry, a.odom_topic, self._odom, 20)
        self.create_subscription(PointCloud2, a.lane_topic, self._lane,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, a.image_topic, self._img,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            "saving every %.1fs from %s -> %s"
            % (a.interval, a.image_topic, a.out))

    def _odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.have_pose = True

    def _lane(self, m):
        # width*height is the point count for an unordered cloud
        self.n_pts = int(m.width) * int(m.height)

    def _img(self, msg):
        t = time.monotonic() - self.t0
        if t - self._last < self.a.interval:
            return
        self._last = t
        try:
            img = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().error("decode failed: %s" % e)
            return
        # The point of the name is that a frame can be found from a drive log,
        # which quotes map coordinates. odom and map coincide here (slam_toolbox
        # publishes an identity-ish correction on an empty map) but say odom,
        # because that is what was measured.
        name = ("f%05d_t%06.1f_x%+07.2f_y%+07.2f_pts%04d.jpg"
                % (self.saved, t, self.x, self.y, min(self.n_pts, 9999)))
        label = ("t=%.1fs  odom (%.2f, %.2f)  lane pts %d"
                 % (t, self.x, self.y, self.n_pts))
        cv2.rectangle(img, (0, 0), (img.shape[1], 18), (0, 0, 0), -1)
        cv2.putText(img, label, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (255, 255, 255), 1)
        cv2.imwrite(os.path.join(self.a.out, name), img,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        self.w.writerow([name, round(t, 1), round(self.x, 2),
                         round(self.y, 2), self.n_pts])
        self.csv.flush()
        self.saved += 1
        if self.saved % 20 == 0:
            self.get_logger().info(
                "%d frames saved, latest %s" % (self.saved, label))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=os.path.expanduser("~/lane_watch"))
    p.add_argument("--image-topic", default="/lane/debug_image",
                   help="the detector's own overlay, masks already drawn")
    p.add_argument("--lane-topic", default="/lane/points")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--interval", type=float, default=2.0,
                   help="wall seconds between saved frames")
    p.add_argument("--seconds", type=float, default=400.0)
    a = p.parse_args()

    rclpy.init()
    n = Watcher(a)
    t_end = time.monotonic() + a.seconds
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    print("saved %d frames to %s" % (n.saved, a.out))
    n.csv.close()
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
