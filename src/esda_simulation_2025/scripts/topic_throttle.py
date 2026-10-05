#!/usr/bin/env python3
"""
Republish a topic at a lower rate, e.g. a viewer-only copy of the VLP-16
cloud for Foxglove over Tailscale:

    /velodyne_points (10 Hz, ~6.7 MB/s) -> /velodyne_points_viz (2 Hz)

Messages are forwarded as raw serialized bytes, never deserialized, so even
large PointCloud2s cost almost no CPU. Stand-in for topic_tools' throttle,
which isn't installed on the Jetson.

Parameters:
  input_topic   topic to read
  output_topic  topic to republish on
  message_type  e.g. sensor_msgs/msg/PointCloud2
  rate          max output rate in Hz
"""

import importlib
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


class TopicThrottle(Node):

    def __init__(self):
        super().__init__('topic_throttle')

        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_viz')
        self.declare_parameter('message_type', 'sensor_msgs/msg/PointCloud2')
        self.declare_parameter('rate', 2.0)

        p = lambda name: self.get_parameter(name).value
        package, _, name = p('message_type').split('/')
        msg_type = getattr(importlib.import_module(f'{package}.msg'), name)
        self.period = 1.0 / float(p('rate'))
        self.last_sent = 0.0

        # Best-effort in, so it matches any publisher; reliable out, so
        # reliable subscribers (ros2 topic hz, RViz defaults) match too.
        self.pub = self.create_publisher(
            msg_type, p('output_topic'),
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(msg_type, p('input_topic'), self.callback,
                                 qos_profile_sensor_data, raw=True)

        self.get_logger().info(
            f"Throttling {p('input_topic')} -> {p('output_topic')} at {p('rate')} Hz")

    def callback(self, serialized):
        now = time.monotonic()
        # 10% slack so a frame arriving a few ms early (10 Hz in, 2 Hz out)
        # isn't skipped, which would drop the output to 1.67 Hz.
        if now - self.last_sent >= 0.9 * self.period:
            self.last_sent = now
            self.pub.publish(serialized)


def main(args=None):
    rclpy.init(args=args)
    node = TopicThrottle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
