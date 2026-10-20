#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from tf2_ros import TransformListener, Buffer
import sys
import numpy as np

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.map = None

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Create a publisher for the frontier list
        self.frontier_list_pub = self.create_publisher(PoseArray, 'frontier_list', 10)
        self.init_timer = self.create_timer(3.0, self.first_detect_frontiers)

        # Transform listener to get robot's current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.min_goal_distance = 0.5  # Minimum distance to consider a frontier as a goal

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        # Create a subscription to listen for when the robot reaches a goal
        self.create_subscription(Bool, "/frontier_reached", self.reached_callback, 10)

        # Create a subscription to execute a function when a pose array is received on the topic "frontier_list"
        self.create_subscription(PoseArray, "frontier_list", self.frontiers_callback_send_goal, 10)



        # Current frontier list
        self.frontier_list = []
        self.current_goal_active = False

    def map_callback(self, msg):
        self.map = msg

        data = np.array(msg.data)

        unknown = np.sum(data == -1)
        free = np.sum(data == 0)
        occupied = np.sum(data > 0)

        self.get_logger().info(
            f"MAP RECEIVED | "
            f"unknown={unknown}, "
            f"free={free}, "
            f"occupied={occupied}"
        )

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)
        except Exception as e:
            self.get_logger().error(f"Failed to get robot pose: {e}")
            return None
        
    def first_detect_frontiers(self):
        """
        Detect frontiers for the first time.
        """
        if self.map is None:
            self.get_logger().info("Waiting for map...")
            return

        self.detect_frontiers()

    def reached_callback(self, msg):
        """
        Callback for when the robot reaches a goal.
        
        :param self: Description
        :param msg: Description
        """
        if msg.data:
            self.get_logger().info("Reached the goal. Detecting new frontiers...")
            self.detect_frontiers()

    def cluster_frontiers(self, frontiers, cluster_distance=0.5):
        # Cluster frontiers based on proximity
        if frontiers is None or len(frontiers) == 0:
            return []
        
        frontiers_set = set(frontiers)
        visited = set()
        clusters = []

        neighbours = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0, 1),
            (1, -1), (1, 0), (1, 1)
        ]

        for frontier in frontiers:
            if frontier in visited:
                continue

            cluster = []
            queue = [frontier]
            visited.add(frontier)

            while queue:
                r, c = queue.pop(0)
                cluster.append((r, c))

                for dr, dc in neighbours:
                    nbr = (r + dr, c + dc)
                    if nbr in frontiers_set and nbr not in visited:
                        visited.add(nbr)
                        queue.append(nbr)

            clusters.append(cluster)
        return clusters
        
    def detect_frontiers(self):
        if self.map is None:
            self.get_logger().info("Map not received yet.")
            return
        

        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            self.get_logger().info("Waiting for robot pose...")
            return
        
        robot_x, robot_y = robot_pose

        grid = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width)) 

        frontiers = []

        for row in range(1, self.map.info.height - 1):
            for col in range(1, self.map.info.width - 1):
                if grid[row, col] == 0: # Free space
                    neighbours = grid[row-1:row+2, col-1:col+2]
                    if np.any(neighbours == -1): # Unknown space
                        # Convert grid coordinates to world coordinates
                        x = col * self.map.info.resolution + self.map.info.origin.position.x
                        y = row * self.map.info.resolution + self.map.info.origin.position.y
                        if np.hypot(x - robot_x, y - robot_y) >= self.min_goal_distance:
                            frontiers.append((x, y))

        # Clustering the frontiers to reduce the number of goals
        clusters = self.cluster_frontiers(frontiers)

        frontier_poses = PoseArray()
        frontier_poses.header.frame_id = "map"
        frontier_poses.header.stamp = self.get_clock().now().to_msg()
        for cluster in clusters:
            if len(clusters) < 5:
                continue

            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]

            mean_row = np.mean(rows)
            mean_col = np.mean(cols)

            # Convert centroid to world coords
            x = mean_col * self.map.info.resolution + self.map.info.origin.position.x
            y = mean_row * self.map.info.resolution + self.map.info.origin.position.y

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # Facing forward
            frontier_poses.poses.append(pose)

        self.frontier_list = frontier_poses
        self.frontier_list_pub.publish(frontier_poses)

    def get_best_frontier(self, frontiers, robot_pose):
        pass  # Placeholder for the actual implementation


    def frontiers_callback_send_goal(self, msg):
        if self.current_goal_active:
            return  # Don't send a new goal if one is already active
        
        if len(msg.poses) == 0:
            self.get_logger().info("No frontiers detected.")
            return
        
        frontier = [pose for pose in msg.poses if np.hypot(pose.position.x, pose.position.y) >= self.min_goal_distance]

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().info("Waiting for robot pose...")
            return
        
        best_centroid = self.get_best_frontier(frontier, robot_pose)

        if best_centroid is None:
            return  # No valid frontier found


        # Don't send goals closer than 0.5 m
        goal_x, goal_y = best_centroid

        goal_distance = np.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1])
        if goal_distance < self.min_goal_distance:
            self.get_logger().info(f"Best frontier is too close ({goal_distance:.2f} m). Not sending goal.")
            return
        
        # Create and publish the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = best_centroid[0]
        goal_pose.pose.position.y = best_centroid[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # Facing forward

        # self.send_goal(goal_pose)
        future = self.nav_client.send_goal_async(goal_pose)
        future.add_done_callback(self.goal_response_callback)
        self.current_goal_active = True

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.current_goal_active = False
        pass

if __name__ == '__main__':
    rclpy.init()
    node = FrontierExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()