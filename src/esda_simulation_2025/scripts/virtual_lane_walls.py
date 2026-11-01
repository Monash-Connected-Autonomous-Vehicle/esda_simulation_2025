#!/usr/bin/env python3
"""
Two virtual walls parallel to the robot's starting heading, lane_half_width
(default 2 m) to its left and right, endless in both directions. Nav2 treats
them as lethal obstacles through its keepout costmap filter, so planned paths
and the controller stay inside the lane.

  anchor = robot pose in the map frame when this node starts
           (or anchor_x/y/yaw if given)
  walls  = the lines through anchor +/- lane_half_width along the anchor's
           left axis, running along the anchor heading

Nav2 applies the keepout filter AFTER inflation, so the walls get no
inflation of their own. Each lethal band therefore extends robot_margin
(default 0.45 m = robot_radius) inward from its line: the robot's CENTRE is
kept that far back, so its body stays inside the lane lines.

"Endless" without an endless grid: the mask is a window_size square centred
on the robot, redrawn whenever the robot gets within a quarter window of its
edge, so the walls are always there wherever the robot drives.

Publishes (latched):
  /virtual_lane_mask             nav_msgs/OccupancyGrid, 100 = wall, -1 = no effect
  /keepout_costmap_filter_info   nav2_msgs/CostmapFilterInfo -> the mask above
  /virtual_lane_walls            visualization_msgs/MarkerArray (RViz/Foxglove)

Uses the same filter info topic as waypoint_navigator_recommendation.py's
travel-history keepout, so don't run both.

    ros2 run esda_simulation_2025 virtual_lane_walls.py
    ros2 run esda_simulation_2025 virtual_lane_walls.py --ros-args -p lane_half_width:=1.5
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

WALL = 100      # -> LETHAL_OBSTACLE through the keepout filter
NO_EFFECT = -1  # unknown in the mask: the filter leaves those cells alone


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rasterise_walls(origin_x, origin_y, size, resolution, anchor, half_width, thickness, margin=0.0):
    """Mask the band from `margin` inside each wall line to thickness/2
    outside it, for an axis-aligned size x size grid whose lower-left corner
    is (origin_x, origin_y)."""
    n = int(round(size / resolution))
    centres = origin_x + (np.arange(n) + 0.5) * resolution
    xs, ys = np.meshgrid(centres, origin_y + (np.arange(n) + 0.5) * resolution)
    ax, ay, yaw = anchor
    # Signed distance to the left of the anchor's heading line.
    lateral = -(xs - ax) * math.sin(yaw) + (ys - ay) * math.cos(yaw)
    distance = np.abs(lateral)
    on_wall = (distance >= half_width - max(margin, thickness / 2.0)) & (distance <= half_width + thickness / 2.0)
    return np.where(on_wall, WALL, NO_EFFECT).astype(np.int8)


class VirtualLaneWalls(Node):

    def __init__(self):
        super().__init__('virtual_lane_walls')

        self.declare_parameter('lane_half_width', 2.0)   # m, anchor to each wall
        self.declare_parameter('wall_thickness', 0.15)   # m; >= 2 cells so paths can't slip through diagonally
        self.declare_parameter('robot_margin', 0.45)     # m kept between robot centre and line (robot_radius)
        self.declare_parameter('window_size', 30.0)      # m, side of the moving mask window
        self.declare_parameter('resolution', 0.05)       # m, match the costmaps
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # Fix the lane instead of taking the robot's pose at start-up.
        self.declare_parameter('use_fixed_anchor', False)
        self.declare_parameter('anchor_x', 0.0)
        self.declare_parameter('anchor_y', 0.0)
        self.declare_parameter('anchor_yaw', 0.0)        # rad

        p = lambda name: self.get_parameter(name).value
        self.half_width = float(p('lane_half_width'))
        self.thickness = max(float(p('wall_thickness')), 2.0 * float(p('resolution')))
        self.margin = float(p('robot_margin'))
        self.window = float(p('window_size'))
        self.resolution = float(p('resolution'))
        self.map_frame = p('map_frame')
        self.base_frame = p('base_frame')
        self.anchor = None
        if p('use_fixed_anchor'):
            self.anchor = (float(p('anchor_x')), float(p('anchor_y')), float(p('anchor_yaw')))
        self.window_origin = None  # lower-left corner of the published window

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/virtual_lane_mask', latched)
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/keepout_costmap_filter_info', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/virtual_lane_walls', latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self.update)
        self.waiting_logged = False

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time(),
                                                timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_from_quaternion(tr.rotation)

    def update(self):
        pose = self.robot_pose()
        if pose is None:
            if not self.waiting_logged:
                self.get_logger().info(
                    f'Waiting for {self.map_frame} -> {self.base_frame} (is SLAM running?)')
                self.waiting_logged = True
            return

        if self.anchor is None:
            self.anchor = pose
            self.get_logger().info(
                f'Lane anchored at the robot: x {pose[0]:.2f} y {pose[1]:.2f} '
                f'heading {math.degrees(pose[2]):.1f} deg; walls {self.half_width:.2f} m left and right '
                f'(robot centre kept {self.margin:.2f} m inside them)')

        # Redraw once the robot is within a quarter window of the edge.
        if self.window_origin is not None:
            cx = self.window_origin[0] + self.window / 2.0
            cy = self.window_origin[1] + self.window / 2.0
            if max(abs(pose[0] - cx), abs(pose[1] - cy)) < self.window / 4.0:
                return
        self.publish(pose)

    def publish(self, pose):
        # Snap the window to the grid so successive masks line up cell-for-cell.
        snap = lambda v: math.floor((v - self.window / 2.0) / self.resolution) * self.resolution
        ox, oy = snap(pose[0]), snap(pose[1])
        self.window_origin = (ox, oy)
        mask = rasterise_walls(ox, oy, self.window, self.resolution, self.anchor,
                               self.half_width, self.thickness, self.margin)
        stamp = self.get_clock().now().to_msg()

        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.header.stamp = stamp
        grid.info.resolution = self.resolution
        grid.info.width = grid.info.height = mask.shape[0]
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0
        grid.data = mask.reshape(-1).tolist()
        self.mask_pub.publish(grid)

        # Republished with every mask so a late-joining costmap gets a
        # consistent pair.
        info = CostmapFilterInfo()
        info.header.frame_id = self.map_frame
        info.header.stamp = stamp
        info.type = 0  # keepout
        info.filter_mask_topic = '/virtual_lane_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

        self.marker_pub.publish(self.markers(stamp))
        self.get_logger().info(f'Lane walls drawn around x {pose[0]:.1f} y {pose[1]:.1f}')

    def markers(self, stamp):
        ax, ay, yaw = self.anchor
        # Long enough to cross the whole window whatever the robot's position.
        cx = self.window_origin[0] + self.window / 2.0
        cy = self.window_origin[1] + self.window / 2.0
        along = (cx - ax) * math.cos(yaw) + (cy - ay) * math.sin(yaw)
        reach = self.window
        out = MarkerArray()
        for i, side in enumerate((1.0, -1.0)):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'virtual_lane_walls'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = self.thickness
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.0, 0.9
            m.pose.orientation.w = 1.0
            for s in (along - reach, along + reach):
                pt = Point()
                pt.x = ax + s * math.cos(yaw) - side * self.half_width * math.sin(yaw)
                pt.y = ay + s * math.sin(yaw) + side * self.half_width * math.cos(yaw)
                m.points.append(pt)
            out.markers.append(m)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = VirtualLaneWalls()
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
