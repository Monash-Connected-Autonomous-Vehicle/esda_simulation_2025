#!/usr/bin/env python3
"""
Waypoint Navigator - Interactive waypoint following with UI
Allows users to set multiple goals in RViz and navigate through them sequentially
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import customtkinter as ctk
import threading
import time
import json
import os
from datetime import datetime


class WaypointNavigator(Node):
    def __init__(self, ui_callback):
        super().__init__('waypoint_navigator')
        
        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscriber for clicked points from RViz (Publish Point button)
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        # Subscriber for robot odometry to record path
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for recorded path visualization
        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)
        
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_navigating = False
        self.current_goal_handle = None
        self.ui_callback = ui_callback
        
        # Path recording
        self.recorded_path = Path()
        self.recorded_path.header.frame_id = 'map'
        self.is_recording = False
        self.last_recorded_point = None
        self.record_distance_threshold = 0.05  # Record point every 5cm
        
        self.get_logger().info('Waypoint Navigator initialized')
        self.get_logger().info('Click "Publish Point" in RViz to add waypoints')
    
    def clicked_point_callback(self, msg):
        """Callback when a point is clicked in RViz using Publish Point button"""
        if not self.is_navigating:
            # Convert PointStamped to PoseStamped (with default orientation)
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose.position = msg.point
            pose_stamped.pose.orientation.w = 1.0  # Default orientation (no rotation)
            
            # Only add waypoints when not actively navigating
            self.waypoints.append(pose_stamped)
            waypoint_num = len(self.waypoints)
            self.ui_callback('waypoint_added', waypoint_num, pose_stamped)
            self.get_logger().info(f'Waypoint {waypoint_num} added at ({msg.point.x:.2f}, {msg.point.y:.2f})')
    
    def odom_callback(self, msg):
        """Record robot position when navigating"""
        if not self.is_recording:
            return
        
        current_point = msg.pose.pose.position
        
        # Only record if moved enough distance from last point
        if self.last_recorded_point is None:
            should_record = True
        else:
            dx = current_point.x - self.last_recorded_point.x
            dy = current_point.y - self.last_recorded_point.y
            distance = (dx**2 + dy**2)**0.5
            should_record = distance >= self.record_distance_threshold
        
        if should_record:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = msg.pose.pose
            self.recorded_path.poses.append(pose_stamped)
            self.last_recorded_point = current_point
            
            # Publish for visualization
            self.recorded_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.recorded_path)
    
    def start_waypoint_navigation(self):
        """Start navigating through all waypoints"""
        if len(self.waypoints) == 0:
            self.get_logger().warn('No waypoints to navigate!')
            self.ui_callback('error', 'No waypoints set')
            return False
        
        if self.is_navigating:
            self.get_logger().warn('Already navigating!')
            return False
        
        self.is_navigating = True
        self.is_recording = True
        self.current_waypoint_index = 0
        self.ui_callback('navigation_started', len(self.waypoints))
        self.get_logger().info(f'Starting navigation through {len(self.waypoints)} waypoints')
        self.get_logger().info('Path recording started')
        
        # Start navigation in a separate thread
        nav_thread = threading.Thread(target=self._navigate_waypoints, daemon=True)
        nav_thread.start()
        return True
    
    def _navigate_waypoints(self):
        """Navigate through all waypoints sequentially"""
        while self.current_waypoint_index < len(self.waypoints) and self.is_navigating:
            waypoint = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}')
            self.ui_callback('navigating_to', self.current_waypoint_index + 1, len(self.waypoints))
            
            # Send goal to Nav2
            success = self._send_goal_and_wait(waypoint)
            
            if success:
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
                self.ui_callback('waypoint_reached', self.current_waypoint_index + 1)
                self.current_waypoint_index += 1
            else:
                self.get_logger().error(f'Failed to reach waypoint {self.current_waypoint_index + 1}')
                self.ui_callback('waypoint_failed', self.current_waypoint_index + 1)
                break
        
        self.is_navigating = False
        self.is_recording = False
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            self.get_logger().info(f'Path recording finished - {len(self.recorded_path.poses)} points recorded')
            self.ui_callback('navigation_complete')
        else:
            self.get_logger().info('Path recording stopped')
            self.ui_callback('navigation_stopped')
    
    def _send_goal_and_wait(self, pose_stamped):
        """Send a single goal to Nav2 and wait for completion"""
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        # Send goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        # Wait for goal to be accepted (polling approach)
        timeout = 10.0
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if not send_goal_future.done():
            self.get_logger().error('Timeout waiting for goal acceptance')
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return False
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted, waiting for completion...')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        
        # Poll for result completion (don't block the spin)
        while not get_result_future.done() and self.is_navigating:
            time.sleep(0.1)
        
        if not get_result_future.done():
            self.get_logger().warn('Navigation was stopped before completion')
            self.current_goal_handle = None
            return False
        
        result = get_result_future.result()
        self.current_goal_handle = None
        
        success = result.status == GoalStatus.STATUS_SUCCEEDED
        if success:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().warn(f'Goal failed with status: {result.status}')
        
        return success
    
    def stop_navigation(self):
        """Stop current navigation"""
        if not self.is_navigating:
            return
        
        self.is_navigating = False
        self.is_recording = False
        
        # Cancel current goal if one is active
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling current goal...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        
        self.get_logger().info('Navigation stopped')
        self.ui_callback('navigation_stopped')
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        if self.is_navigating:
            self.stop_navigation()
        self.waypoints.clear()
        self.current_waypoint_index = 0
        self.get_logger().info('All waypoints cleared')
        self.ui_callback('waypoints_cleared')
    
    def save_recorded_path(self, filename):
        """Save the recorded path to a JSON file"""
        if len(self.recorded_path.poses) == 0:
            self.get_logger().warn('No path to save')
            return False
        
        path_data = {
            'timestamp': datetime.now().isoformat(),
            'num_points': len(self.recorded_path.poses),
            'frame_id': self.recorded_path.header.frame_id,
            'poses': []
        }
        
        for pose_stamped in self.recorded_path.poses:
            path_data['poses'].append({
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'z': pose_stamped.pose.position.z,
                'qx': pose_stamped.pose.orientation.x,
                'qy': pose_stamped.pose.orientation.y,
                'qz': pose_stamped.pose.orientation.z,
                'qw': pose_stamped.pose.orientation.w
            })
        
        try:
            with open(filename, 'w') as f:
                json.dump(path_data, f, indent=2)
            self.get_logger().info(f'Path saved to {filename}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save path: {str(e)}')
            return False
    
    def clear_recorded_path(self):
        """Clear the recorded path"""
        self.recorded_path.poses.clear()
        self.last_recorded_point = None
        self.get_logger().info('Recorded path cleared')
        self.ui_callback('path_cleared')
    
    def remove_last_waypoint(self):
        """Remove the last waypoint"""
        if len(self.waypoints) > 0 and not self.is_navigating:
            self.waypoints.pop()
            self.get_logger().info(f'Last waypoint removed. {len(self.waypoints)} remaining')
            self.ui_callback('waypoint_removed', len(self.waypoints))
            return True
        return False


class WaypointNavigatorUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        self.title("Waypoint Navigator")
        self.geometry("600x850")
        
        # Styling to match the main UI
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.configure(bg="#10131a")
        
        # Colors
        self.accent_purple = "#7C3AED"
        self.accent_blue = "#00FFF7"
        self.accent_green = "#00FFB2"
        self.accent_red = "#FF0059"
        self.bg_dark = "#10131a"
        self.bg_panel = "#181C25"
        self.fg_text = "#E0E6F8"
        
        # ROS2 Node (will be initialized in separate thread)
        self.nav_node = None
        self.ros_thread = None
        
        # UI Layout
        self.grid_columnconfigure(0, weight=1)
        
        # Title
        title = ctk.CTkLabel(self, text="WAYPOINT NAVIGATOR", 
                            font=("Orbitron", 24, "bold"), 
                            text_color=self.accent_purple)
        title.grid(row=0, column=0, pady=15, padx=20)
        
        # Instructions
        instructions = ctk.CTkLabel(
            self, 
            text="1. Click 'Start ROS Node'\n2. Click 'Publish Point' in RViz to add waypoints\n3. Click 'Start Navigation' to begin",
            font=("Orbitron", 11),
            text_color=self.accent_blue,
            justify="left"
        )
        instructions.grid(row=1, column=0, pady=10, padx=20)
        
        # ROS Node Control
        node_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        node_frame.grid(row=2, column=0, pady=10, padx=20, sticky="ew")
        
        self.node_button = ctk.CTkButton(
            node_frame,
            text="Start ROS Node",
            command=self.toggle_ros_node,
            fg_color=self.accent_purple,
            hover_color="#5F27CD",
            font=("Orbitron", 14, "bold"),
            height=40
        )
        self.node_button.pack(pady=10, padx=10, fill="x")
        
        # Waypoint List
        list_label = ctk.CTkLabel(self, text="Waypoints", 
                                 font=("Orbitron", 16, "bold"),
                                 text_color=self.accent_blue)
        list_label.grid(row=3, column=0, pady=(15, 5))
        
        # Scrollable frame for waypoints
        self.waypoint_frame = ctk.CTkScrollableFrame(
            self,
            fg_color=self.bg_panel,
            height=250
        )
        self.waypoint_frame.grid(row=4, column=0, pady=5, padx=20, sticky="ew")
        self.waypoint_frame.grid_columnconfigure(0, weight=1)
        
        self.waypoint_labels = []
        self.no_waypoints_label = ctk.CTkLabel(
            self.waypoint_frame,
            text="No waypoints added yet\nUse 'Publish Point' in RViz to add waypoints",
            font=("Orbitron", 10),
            text_color=self.fg_text
        )
        self.no_waypoints_label.grid(row=0, column=0, pady=20)
        
        # Status
        status_label = ctk.CTkLabel(self, text="Status", 
                                   font=("Orbitron", 14, "bold"),
                                   text_color=self.accent_blue)
        status_label.grid(row=5, column=0, pady=(15, 5))
        
        self.status_text = ctk.CTkLabel(
            self,
            text="Ready - Start ROS node to begin",
            font=("Orbitron", 11),
            text_color=self.fg_text,
            wraplength=500
        )
        self.status_text.grid(row=6, column=0, pady=5)
        
        # Control Buttons
        control_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        control_frame.grid(row=7, column=0, pady=15, padx=20, sticky="ew")
        control_frame.grid_columnconfigure((0, 1), weight=1)
        
        self.start_button = ctk.CTkButton(
            control_frame,
            text="Start Navigation",
            command=self.start_navigation,
            fg_color=self.accent_green,
            hover_color="#00B894",
            font=("Orbitron", 14, "bold"),
            height=45,
            state="disabled"
        )
        self.start_button.grid(row=0, column=0, columnspan=2, pady=5, padx=5, sticky="ew")
        
        self.stop_button = ctk.CTkButton(
            control_frame,
            text="Stop Navigation",
            command=self.stop_navigation,
            fg_color=self.accent_red,
            hover_color="#C0392B",
            font=("Orbitron", 12, "bold"),
            height=40,
            state="disabled"
        )
        self.stop_button.grid(row=1, column=0, columnspan=2, pady=5, padx=5, sticky="ew")
        
        self.remove_button = ctk.CTkButton(
            control_frame,
            text="Remove Last",
            command=self.remove_last_waypoint,
            fg_color=self.accent_purple,
            hover_color="#5F27CD",
            font=("Orbitron", 11),
            state="disabled"
        )
        self.remove_button.grid(row=2, column=0, pady=5, padx=5, sticky="ew")
        
        self.clear_button = ctk.CTkButton(
            control_frame,
            text="Clear All",
            command=self.clear_waypoints,
            fg_color=self.accent_purple,
            hover_color="#5F27CD",
            font=("Orbitron", 11),
            state="disabled"
        )
        self.clear_button.grid(row=2, column=1, pady=5, padx=5, sticky="ew")
        
        # Path Recording Section
        path_label = ctk.CTkLabel(self, text="Path Recording", 
                                 font=("Orbitron", 14, "bold"),
                                 text_color=self.accent_blue)
        path_label.grid(row=8, column=0, pady=(15, 5))
        
        self.path_info_label = ctk.CTkLabel(
            self,
            text="No path recorded yet",
            font=("Orbitron", 10),
            text_color=self.fg_text
        )
        self.path_info_label.grid(row=9, column=0, pady=3)
        
        path_control_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        path_control_frame.grid(row=10, column=0, pady=10, padx=20, sticky="ew")
        path_control_frame.grid_columnconfigure((0, 1), weight=1)
        
        self.save_path_button = ctk.CTkButton(
            path_control_frame,
            text="💾 Save Path",
            command=self.save_path,
            fg_color=self.accent_green,
            hover_color="#00B894",
            font=("Orbitron", 12, "bold"),
            state="disabled"
        )
        self.save_path_button.grid(row=0, column=0, pady=5, padx=5, sticky="ew")
        
        self.clear_path_button = ctk.CTkButton(
            path_control_frame,
            text="Clear Path",
            command=self.clear_path,
            fg_color=self.accent_purple,
            hover_color="#5F27CD",
            font=("Orbitron", 12, "bold"),
            state="disabled"
        )
        self.clear_path_button.grid(row=0, column=1, pady=5, padx=5, sticky="ew")
        
        self.ros_running = False
    
    def toggle_ros_node(self):
        """Start or stop ROS node"""
        if not self.ros_running:
            self.start_ros_node()
        else:
            self.stop_ros_node()
    
    def start_ros_node(self):
        """Initialize and start ROS2 node"""
        try:
            rclpy.init()
            self.nav_node = WaypointNavigator(self.handle_nav_callback)
            
            # Spin ROS in separate thread
            self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.ros_thread.start()
            
            self.ros_running = True
            self.node_button.configure(text="Stop ROS Node", fg_color=self.accent_red)
            self.start_button.configure(state="normal")
            self.remove_button.configure(state="normal")
            self.clear_button.configure(state="normal")
            self.status_text.configure(
                text="ROS Node Running - Add waypoints using 'Publish Point' in RViz",
                text_color=self.accent_green
            )
        except Exception as e:
            self.status_text.configure(
                text=f"Error starting ROS: {str(e)}",
                text_color=self.accent_red
            )
    
    def stop_ros_node(self):
        """Stop ROS2 node"""
        if self.nav_node:
            self.nav_node.destroy_node()
        if self.ros_running:
            rclpy.shutdown()
        self.ros_running = False
        self.node_button.configure(text="Start ROS Node", fg_color=self.accent_purple)
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="disabled")
        self.remove_button.configure(state="disabled")
        self.clear_button.configure(state="disabled")
        self.status_text.configure(
            text="ROS Node Stopped",
            text_color=self.fg_text
        )
    
    def spin_ros(self):
        """Spin ROS node in separate thread"""
        try:
            rclpy.spin(self.nav_node)
        except Exception as e:
            print(f"ROS spin error: {e}")
    
    def handle_nav_callback(self, event_type, *args):
        """Handle callbacks from navigation node"""
        self.after(0, self._process_nav_event, event_type, *args)
    
    def _process_nav_event(self, event_type, *args):
        """Process navigation events in UI thread"""
        if event_type == 'waypoint_added':
            waypoint_num, pose = args
            self.add_waypoint_to_list(waypoint_num, pose)
            self.status_text.configure(
                text=f"Waypoint {waypoint_num} added at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})",
                text_color=self.accent_green
            )
        
        elif event_type == 'navigation_started':
            total = args[0]
            self.start_button.configure(state="disabled")
            self.stop_button.configure(state="normal")
            self.remove_button.configure(state="disabled")
            self.clear_button.configure(state="disabled")
            self.status_text.configure(
                text=f"Navigation started - {total} waypoints to visit",
                text_color=self.accent_green
            )
        
        elif event_type == 'navigating_to':
            current, total = args
            self.status_text.configure(
                text=f"Navigating to waypoint {current}/{total}...",
                text_color=self.accent_blue
            )
            self.highlight_current_waypoint(current - 1)
        
        elif event_type == 'waypoint_reached':
            waypoint_num = args[0]
            self.mark_waypoint_complete(waypoint_num - 1)
            self.status_text.configure(
                text=f"Waypoint {waypoint_num} reached!",
                text_color=self.accent_green
            )
        
        elif event_type == 'waypoint_failed':
            waypoint_num = args[0]
            self.status_text.configure(
                text=f"Failed to reach waypoint {waypoint_num}",
                text_color=self.accent_red
            )
            self.reset_navigation_ui()
        
        elif event_type == 'navigation_complete':
            num_points = len(self.nav_node.recorded_path.poses) if self.nav_node else 0
            self.status_text.configure(
                text="All waypoints completed! 🎉",
                text_color=self.accent_green
            )
            self.reset_navigation_ui()
            if num_points > 0:
                self.path_info_label.configure(
                    text=f"Path recorded: {num_points} points",
                    text_color=self.accent_green
                )
                self.save_path_button.configure(state="normal")
                self.clear_path_button.configure(state="normal")
        
        elif event_type == 'navigation_stopped':
            num_points = len(self.nav_node.recorded_path.poses) if self.nav_node else 0
            self.status_text.configure(
                text="Navigation stopped",
                text_color=self.fg_text
            )
            self.reset_navigation_ui()
            if num_points > 0:
                self.path_info_label.configure(
                    text=f"Path recorded (partial): {num_points} points",
                    text_color=self.fg_text
                )
                self.save_path_button.configure(state="normal")
                self.clear_path_button.configure(state="normal")
        
        elif event_type == 'waypoints_cleared':
            self.clear_waypoint_list()
            self.status_text.configure(
                text="All waypoints cleared",
                text_color=self.fg_text
            )
        
        elif event_type == 'waypoint_removed':
            remaining = args[0]
            self.remove_last_waypoint_from_list()
            self.status_text.configure(
                text=f"Last waypoint removed - {remaining} remaining",
                text_color=self.fg_text
            )
        
        elif event_type == 'error':
            message = args[0]
            self.status_text.configure(
                text=f"Error: {message}",
                text_color=self.accent_red
            )
        
        elif event_type == 'path_cleared':
            self.path_info_label.configure(text="No path recorded yet")
            self.save_path_button.configure(state="disabled")
            self.clear_path_button.configure(state="disabled")
    
    def add_waypoint_to_list(self, num, pose):
        """Add waypoint to UI list"""
        if self.no_waypoints_label.winfo_exists():
            self.no_waypoints_label.grid_forget()
        
        x = pose.pose.position.x
        y = pose.pose.position.y
        
        wp_frame = ctk.CTkFrame(self.waypoint_frame, fg_color=self.bg_dark)
        wp_frame.grid(row=num-1, column=0, pady=3, padx=5, sticky="ew")
        wp_frame.grid_columnconfigure(1, weight=1)
        
        num_label = ctk.CTkLabel(
            wp_frame,
            text=f"{num}",
            font=("Orbitron", 14, "bold"),
            text_color=self.accent_purple,
            width=30
        )
        num_label.grid(row=0, column=0, padx=10, pady=8)
        
        pos_label = ctk.CTkLabel(
            wp_frame,
            text=f"Position: ({x:.2f}, {y:.2f})",
            font=("Orbitron", 11),
            text_color=self.fg_text,
            anchor="w"
        )
        pos_label.grid(row=0, column=1, padx=10, pady=8, sticky="w")
        
        status_label = ctk.CTkLabel(
            wp_frame,
            text="⏳ Pending",
            font=("Orbitron", 10),
            text_color="#F1C40F",
            width=80
        )
        status_label.grid(row=0, column=2, padx=10, pady=8)
        
        self.waypoint_labels.append({
            'frame': wp_frame,
            'status': status_label
        })
    
    def highlight_current_waypoint(self, index):
        """Highlight the current waypoint being navigated to"""
        if index < len(self.waypoint_labels):
            self.waypoint_labels[index]['status'].configure(
                text="🚗 Active",
                text_color=self.accent_blue
            )
            self.waypoint_labels[index]['frame'].configure(fg_color="#2C3E50")
    
    def mark_waypoint_complete(self, index):
        """Mark waypoint as completed"""
        if index < len(self.waypoint_labels):
            self.waypoint_labels[index]['status'].configure(
                text="✓ Done",
                text_color=self.accent_green
            )
            self.waypoint_labels[index]['frame'].configure(fg_color=self.bg_dark)
    
    def clear_waypoint_list(self):
        """Clear all waypoints from UI"""
        for wp in self.waypoint_labels:
            wp['frame'].destroy()
        self.waypoint_labels.clear()
        self.no_waypoints_label.grid(row=0, column=0, pady=20)
    
    def remove_last_waypoint_from_list(self):
        """Remove last waypoint from UI"""
        if len(self.waypoint_labels) > 0:
            last = self.waypoint_labels.pop()
            last['frame'].destroy()
        
        if len(self.waypoint_labels) == 0:
            self.no_waypoints_label.grid(row=0, column=0, pady=20)
    
    def reset_navigation_ui(self):
        """Reset UI after navigation completes or stops"""
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.remove_button.configure(state="normal")
        self.clear_button.configure(state="normal")
    
    def start_navigation(self):
        """Start waypoint navigation"""
        if self.nav_node:
            # Clear previous path recording
            self.nav_node.clear_recorded_path()
            self.path_info_label.configure(text="Recording path...", text_color=self.accent_blue)
            self.save_path_button.configure(state="disabled")
            self.clear_path_button.configure(state="disabled")
            self.nav_node.start_waypoint_navigation()
    
    def stop_navigation(self):
        """Stop navigation"""
        if self.nav_node:
            self.nav_node.stop_navigation()
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        if self.nav_node:
            self.nav_node.clear_waypoints()
    
    def remove_last_waypoint(self):
        """Remove last waypoint"""
        if self.nav_node:
            self.nav_node.remove_last_waypoint()
    
    def save_path(self):
        """Save the recorded path to a file"""
        if not self.nav_node:
            return
        
        # Create default filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"recorded_path_{timestamp}.json"
        
        # Use file dialog to choose save location
        from tkinter import filedialog
        filename = filedialog.asksaveasfilename(
            title="Save Recorded Path",
            defaultextension=".json",
            initialfile=default_filename,
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
        )
        
        if filename:
            success = self.nav_node.save_recorded_path(filename)
            if success:
                self.status_text.configure(
                    text=f"Path saved successfully to {os.path.basename(filename)}",
                    text_color=self.accent_green
                )
            else:
                self.status_text.configure(
                    text="Failed to save path",
                    text_color=self.accent_red
                )
    
    def clear_path(self):
        """Clear the recorded path"""
        if self.nav_node:
            self.nav_node.clear_recorded_path()
            self.status_text.configure(
                text="Recorded path cleared",
                text_color=self.fg_text
            )
    
    def on_closing(self):
        """Handle window closing"""
        if self.ros_running:
            self.stop_ros_node()
        self.destroy()


if __name__ == "__main__":
    app = WaypointNavigatorUI()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
