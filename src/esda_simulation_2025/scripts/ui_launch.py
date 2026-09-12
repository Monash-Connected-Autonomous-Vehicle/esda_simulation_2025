# This code is to launch the UI for the real robot

import customtkinter as ctk # type: ignore
import subprocess
import os
import signal
import time
import glob
import threading
from tkinter import filedialog

class SimManager(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("ESDA SIMULATION SUITE")
        self.geometry("700x700")

        # Futuristic Appearance
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.configure(bg="#10131a")

        # Try to use Orbitron font everywhere, fallback to Roboto if not available
        # (Orbitron is a free Google font, user may need to install it for best effect)

        # Accent colors
        self.accent_blue = "#00FFF7"
        self.accent_purple = "#7C3AED"
        self.accent_green = "#00FFB2"
        self.accent_orange = "#FF8C00"
        self.accent_red = "#FF0059"
        self.bg_dark = "#10131a"
        self.bg_panel = "#181C25"
        self.fg_text = "#E0E6F8"
        self.fg_dim = "#7A7F9A"

        self.processes = {}
        # Determine workspace root dynamically
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if "src/esda_simulation_2025/scripts" in script_dir:
            self.workspace_root = os.path.abspath(os.path.join(script_dir, "../../.."))
        elif "lib/esda_simulation_2025" in script_dir:
            self.workspace_root = os.path.abspath(os.path.join(script_dir, "../../../.."))
        else:
            self.workspace_root = os.getcwd()
        
        # Scan for available files
        self.world_files = self.scan_world_files()
        self.costmap_files = self.scan_costmap_files()
        
        # Default selections
        default_world = "igvc.sdf" if any(os.path.basename(f) == "igvc.sdf" for f in self.world_files) else "[Select World File]"
        self.selected_world = ctk.StringVar(value=default_world)
        self.selected_costmap = ctk.StringVar(value="[New Costmap]")

        # UI Layout
        self.grid_columnconfigure(0, weight=1)
        
        # Title
        self.label = ctk.CTkLabel(self, text="ESDA SIMULATION SUITE", font=("Orbitron", 24, "bold"), text_color=self.accent_blue, bg_color=self.bg_dark)
        self.label.grid(row=0, column=0, pady=12)

        # Build Section
        self.build_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.build_frame.grid(row=1, column=0, pady=6, padx=12, sticky="ew")
        
        self.build_button = ctk.CTkButton(self.build_frame, text="1. Colcon Build (Required for changes)", 
                          command=self.build_workspace, 
                          fg_color=self.accent_purple, hover_color="#5F27CD", font=("Orbitron", 14, "bold"), text_color=self.bg_dark)
        self.build_button.pack(padx=8, pady=8, side="left", expand=True, fill="x")

        # File Selection Section
        self.file_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.file_frame.grid(row=2, column=0, pady=6, padx=12, sticky="ew")
        self.file_frame.grid_columnconfigure(1, weight=1)
        
        # World SDF File Selection
        self.world_label = ctk.CTkLabel(self.file_frame, text="World SDF:", font=("Orbitron", 11), text_color=self.accent_purple, bg_color=self.bg_panel)
        self.world_label.grid(row=0, column=0, padx=6, pady=3, sticky="w")
        
        self.world_dropdown = ctk.CTkOptionMenu(self.file_frame, variable=self.selected_world, 
                            values=[os.path.basename(f) for f in self.world_files],
                            width=280, fg_color=self.bg_dark, button_color=self.accent_purple, text_color=self.fg_text)
        self.world_dropdown.grid(row=0, column=1, padx=6, pady=3, sticky="ew")
        
        self.world_browse_button = ctk.CTkButton(self.file_frame, text="Browse...", command=self.browse_world_file,
                                                 width=80, fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.world_browse_button.grid(row=0, column=2, padx=6, pady=3)
        
        
        # Costmap File Selection
        self.costmap_label = ctk.CTkLabel(self.file_frame, text="Costmap YAML:", font=("Orbitron", 11), text_color=self.accent_purple, bg_color=self.bg_panel)
        self.costmap_label.grid(row=1, column=0, padx=6, pady=3, sticky="w")
        
        costmap_options = ["[New Costmap]"] + [os.path.basename(f) for f in self.costmap_files]
        self.costmap_dropdown = ctk.CTkOptionMenu(self.file_frame, variable=self.selected_costmap,
                              values=costmap_options,
                              width=280, fg_color=self.bg_dark, button_color=self.accent_purple, text_color=self.fg_text)
        self.costmap_dropdown.grid(row=1, column=1, padx=6, pady=3, sticky="ew")
        
        self.costmap_browse_button = ctk.CTkButton(self.file_frame, text="Browse...", command=self.browse_costmap_file,
                                                   width=80, fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.costmap_browse_button.grid(row=1, column=2, padx=6, pady=3)


        # Simulation Section (LIDAR and Lane Detection side by side)
        self.sim_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.sim_frame.grid(row=3, column=0, pady=6, padx=16, sticky="ew")
        self.sim_frame.grid_columnconfigure(0, weight=1)
        self.sim_frame.grid_columnconfigure(1, weight=1)
        self.sim_frame.grid_columnconfigure(2, weight=1)

        self.lidar_var = ctk.BooleanVar(value=True)
        self.lidar_check = ctk.CTkCheckBox(self.sim_frame, text="Enable LIDAR", variable=self.lidar_var, font=("Orbitron", 14), text_color=self.accent_blue, bg_color=self.bg_panel)
        self.lidar_check.grid(row=0, column=0, padx=10, pady=6, sticky="w")

        self.dual_camera_var = ctk.BooleanVar(value=False)
        self.dual_camera_check = ctk.CTkCheckBox(self.sim_frame, text="Dual Side Cameras (Test)", variable=self.dual_camera_var, font=("Orbitron", 14), text_color=self.accent_blue, bg_color=self.bg_panel)
        self.dual_camera_check.grid(row=2, column=0, padx=10, pady=(0, 8), sticky="w")

        self.lane_detection_var = ctk.BooleanVar(value=True)
        self.lane_detection_check = ctk.CTkCheckBox(self.sim_frame, text="Enable Lane Detection", variable=self.lane_detection_var, font=("Orbitron", 14), text_color=self.accent_purple, bg_color=self.bg_panel)
        self.lane_detection_check.grid(row=0, column=1, padx=10, pady=6, sticky="w")

        self.lane_detector_mode = ctk.StringVar(value="FCN")
        self.lane_mode_dropdown = ctk.CTkOptionMenu(
            self.sim_frame,
            variable=self.lane_detector_mode,
            values=["Regular", "FCN", "TwinLiteNet+"],
            width=130,
            fg_color=self.bg_dark,
            button_color=self.accent_purple,
            text_color=self.fg_text
        )
        self.lane_mode_dropdown.grid(row=0, column=2, padx=10, pady=6, sticky="e")

        # Lane detection visualization dropdown
        self.lane_visualization = ctk.StringVar(value="Visualization On")

        self.lane_visualization_dropdown = ctk.CTkOptionMenu(
            self.sim_frame,
            variable=self.lane_visualization,
            values=["Visualization On", "Visualization Off"],
            width=160,
            fg_color=self.bg_dark,
            button_color=self.accent_purple,
            text_color=self.fg_text
        )

        self.lane_visualization_dropdown.grid(
            row=1,
            column=0,
            columnspan=2,
            padx=10,
            pady=(0, 8),
            sticky="ew"
        )

        self.sim_button = ctk.CTkButton(self.sim_frame, text="2. Launch Simulation", command=self.toggle_sim, font=("Orbitron", 16, "bold"), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.sim_button.grid(row=0, column=3, padx=10, pady=6, sticky="e")

        self.lane_detection_button = ctk.CTkButton(self.sim_frame, text="Launch Lane Detection", command=self.toggle_lane_detection, font=("Orbitron", 13, "bold"), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.lane_detection_button.grid(row=1, column=2, columnspan=2, padx=10, pady=(0, 8), sticky="ew")

        # Remove SLAM Options Section (now merged)

        # Modules Label
        self.mod_label = ctk.CTkLabel(self, text="Navigation & SLAM Modules", font=("Orbitron", 13, "italic"), text_color=self.accent_blue, bg_color=self.bg_dark)
        self.mod_label.grid(row=5, column=0, pady=(8, 2))

        # Modules Section
        self.modules_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.modules_frame.grid(row=6, column=0, pady=6, padx=12, sticky="ew")


        # EKF (robot localization) now launches automatically as part of the
        # sim (see launch_sim.launch.py) - a separate button here would let
        # you spawn a second, conflicting ekf_filter_node instance.

        self.slam_button = ctk.CTkButton(self.modules_frame, text="Launch SLAM", command=self.toggle_slam, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.slam_button.grid(row=0, column=0, padx=6, pady=6, sticky="ew")
        self.amcl_button = ctk.CTkButton(self.modules_frame, text="Launch AMCL", command=self.toggle_amcl, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.amcl_button.grid(row=0, column=1, padx=6, pady=6, sticky="ew")
        self.nav_button = ctk.CTkButton(self.modules_frame, text="Launch Nav2", command=self.toggle_nav, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.nav_button.grid(row=1, column=0, padx=6, pady=6, sticky="ew")
        self.rviz_button = ctk.CTkButton(self.modules_frame, text="Launch RViz2", command=self.toggle_rviz, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.rviz_button.grid(row=1, column=1, padx=6, pady=6, sticky="ew")
        self.follow_the_gap_button = ctk.CTkButton(self.modules_frame, text="Follow the Gap Algorithm", command=self.toggle_follow_the_gap, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.follow_the_gap_button.grid(row=2, column=0, padx=6, pady=6, sticky="ew")
        self.track_follower_button = ctk.CTkButton(self.modules_frame, text="Track Follower Algorithm", command=self.launch_track_follower, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.track_follower_button.grid(row=2, column=1, padx=6, pady=6, sticky="ew")
        self.modules_frame.grid_columnconfigure((0,1), weight=1)


        self.behaviour_tree_button = ctk.CTkButton(self.modules_frame, text="Launch Behaviour Tree", command=self.launch_behaviour_tree, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.behaviour_tree_button.grid(row=3, column=0, columnspan=2, padx=6, pady=6, sticky="ew")
        self.modules_frame.grid_rowconfigure((0,1,2,3), weight=1)

        # Teleop and Waypoint Section
        self.teleop_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.teleop_frame.grid(row=7, column=0, pady=6, padx=12, sticky="ew")
        self.teleop_frame.grid_columnconfigure((0,1), weight=1)
        
        self.teleop_button = ctk.CTkButton(self.teleop_frame, text="WASD Teleop", command=self.toggle_teleop,
                           fg_color=self.accent_purple, hover_color="#5F27CD", font=("Orbitron", 14, "bold"), text_color=self.bg_dark)
        self.teleop_button.grid(row=0, column=0, pady=6, padx=6, sticky="ew")
        
        print("WDAWDAWDA")
        self.waypoint_button = ctk.CTkButton(self.teleop_frame, text="Waypoint Nav", command=self.launch_waypoint_navigator,
                         fg_color=self.accent_purple, hover_color="#5F27CD", font=("Orbitron", 14, "bold"), text_color=self.bg_dark)
        self.waypoint_button.grid(row=0, column=1, pady=6, padx=6, sticky="ew")
        
        # Diagnostics Section
        self.diag_button = ctk.CTkButton(self, text="Check /clock Topic (Diagnostics)", command=self.check_clock,
                         fg_color=self.accent_purple, hover_color="#5F27CD", font=("Orbitron", 11), text_color=self.bg_dark)
        self.diag_button.grid(row=8, column=0, pady=3, padx=12, sticky="ew")

        # Emergency Section
        self.kill_button = ctk.CTkButton(self, text="KILL ALL PROCESSES & RESET GAZEBO", command=self.kill_all, 
                         fg_color=self.accent_red, hover_color="#7B241C", font=("Orbitron", 12, "bold"), text_color=self.bg_dark)
        self.kill_button.grid(row=9, column=0, pady=10, padx=12, sticky="ew")

        self.status_label = ctk.CTkLabel(self, text="System Ready", text_color=self.fg_dim, font=("Orbitron", 10), bg_color=self.bg_dark)
        self.status_label.grid(row=10, column=0, pady=4)

        # Check for xterm
        self.check_xterm()

        # Store original colors
        self.default_colors = {
            "SIM": self.sim_button.cget("fg_color"),
            "SLAM": self.slam_button.cget("fg_color"),
            "AMCL": self.amcl_button.cget("fg_color"),
            "NAV": self.nav_button.cget("fg_color"),
            "RVIZ": self.rviz_button.cget("fg_color"),
            "TELEOP": self.teleop_button.cget("fg_color"),
            "LANE": self.lane_detection_button.cget("fg_color"),
        }

    def scan_world_files(self):
        """Scan the worlds directory for .sdf files"""
        worlds_dir = f"{self.workspace_root}/src/esda_simulation_2025/worlds"
        # Get all .sdf files directly in the worlds directory (not in subdirectories)
        world_files = glob.glob(f"{worlds_dir}/*.sdf")
        return sorted(world_files) if world_files else []

    def scan_costmap_files(self):
        """Scan the maps directory for .yaml files"""
        maps_dir = f"{self.workspace_root}/src/esda_simulation_2025/maps"
        costmap_files = glob.glob(f"{maps_dir}/*.yaml")
        return sorted(costmap_files) if costmap_files else []
    
    def browse_world_file(self):
        """Open a file browser to select a world file"""
        initial_dir = f"{self.workspace_root}/src/esda_simulation_2025/worlds"
        filename = filedialog.askopenfilename(
            title="Select World File",
            initialdir=initial_dir if os.path.exists(initial_dir) else self.workspace_root,
            filetypes=[("SDF Files", "*.sdf"), ("World Files", "*.world"), ("All Files", "*.*")]
        )
        if filename:
            # Add to world_files list if not already there
            if filename not in self.world_files:
                self.world_files.append(filename)
                self.world_files.sort()
                # Update dropdown values
                self.world_dropdown.configure(values=[os.path.basename(f) for f in self.world_files])
            # Set as selected
            self.selected_world.set(os.path.basename(filename))
            self.status_label.configure(text=f"Selected: {os.path.basename(filename)}", text_color="#2ECC71")
    
    def browse_costmap_file(self):
        """Open a file browser to select a costmap file"""
        initial_dir = f"{self.workspace_root}/src/esda_simulation_2025/maps"
        filename = filedialog.askopenfilename(
            title="Select Costmap File",
            initialdir=initial_dir if os.path.exists(initial_dir) else self.workspace_root,
            filetypes=[("YAML Files", "*.yaml"), ("All Files", "*.*")]
        )
        if filename:
            # Add to costmap_files list if not already there
            if filename not in self.costmap_files:
                self.costmap_files.append(filename)
                self.costmap_files.sort()
                # Update dropdown values
                costmap_options = ["[New Costmap]"] + [os.path.basename(f) for f in self.costmap_files]
                self.costmap_dropdown.configure(values=costmap_options)
            # Set as selected
            self.selected_costmap.set(os.path.basename(filename))
            self.status_label.configure(text=f"Selected: {os.path.basename(filename)}", text_color="#2ECC71")

    def run_in_terminal(self, name, command):
        if name in self.processes and self.processes[name].poll() is None:
            self.stop_process(name)
            return

        # Try to find ROS_DISTRO or default to humble
        ros_distro = os.environ.get("ROS_DISTRO", "humble")
        ros_setup = f"/opt/ros/{ros_distro}/setup.bash"
        if not os.path.exists(ros_setup):
            ros_setup_cmd = "source /opt/ros/*/setup.bash 2>/dev/null || true"
        else:
            ros_setup_cmd = f"source {ros_setup}"

        # Setup local workspace
        local_setup = os.path.join(self.workspace_root, "install/setup.bash")
        if os.path.exists(local_setup):
            local_setup_cmd = f"source {local_setup}"
        else:
            local_setup_cmd = ":" # No-op

        # Prepare the command to source ROS and our workspace
        # We also disable SHM transport to avoid FastDDS errors in Docker
        full_command = (f'xterm -T "{name}" -geometry 100x30 -e "bash -c \\"'
                        f'export FASTRTPS_DEFAULT_PROFILES_FILE={self.workspace_root}/src/esda_simulation_2025/config/fastdds_noshm.xml && '
                        f'{ros_setup_cmd} && '
                        f'{local_setup_cmd} && '
                        f'echo Starting {name}... && '
                        f'{command}; '
                        f'echo; echo Process finished. Press Enter to close window...; read\\""')

        try:
            process = subprocess.Popen(full_command, shell=True, preexec_fn=os.setsid)
            self.processes[name] = process
            self.update_ui_state(name, True)
            self.status_label.configure(text=f"Started {name}", text_color="#2ECC71")
        except Exception as e:
            self.status_label.configure(text=f"Error: {str(e)}", text_color="#E74C3C")

    def stop_process(self, name):
        if name in self.processes:
            p = self.processes[name]
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except:
                    pass
            del self.processes[name]
            self.update_ui_state(name, False)
            self.status_label.configure(text=f"Stopped {name}", text_color="#BDC3C7")

    def update_ui_state(self, name, running):
        color = "#C0392B" if running else self.default_colors.get(name)
        
        if name == "SIM": self.sim_button.configure(fg_color=color)
        elif name == "SLAM": self.slam_button.configure(fg_color=color)
        elif name == "AMCL": self.amcl_button.configure(fg_color=color)
        elif name == "NAV": self.nav_button.configure(fg_color=color)
        elif name == "RVIZ": self.rviz_button.configure(fg_color=color)
        elif name == "TELEOP": self.teleop_button.configure(fg_color=color)
        elif name == "LANE": self.lane_detection_button.configure(fg_color=color)

    def check_xterm(self):
        """Check if xterm is installed"""
        try:
            subprocess.run(["which", "xterm"], check=True, capture_output=True)
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.after(1000, lambda: self.status_label.configure(
                text="WARNING: 'xterm' not found. Install with: sudo apt install xterm", 
                text_color="#FF0059"
            ))

    def build_workspace(self):
        self.status_label.configure(text="Building... Check terminal window", text_color="#F1C40F")
        self.update()
        
        cmd = f'xterm -T "Build Process" -e "bash -c \\"cd {self.workspace_root} && colcon build --packages-select esda_simulation_2025; echo; echo Done. Press Enter to close.; read\\""'
        proc = subprocess.run(cmd, shell=True)
        
        self.status_label.configure(text="Build attempt finished", text_color="#BDC3C7")

    def toggle_sim(self):
        lidar = "true" if self.lidar_var.get() else "false"
        robot_model = "robot_dual_camera.urdf.xacro" if self.dual_camera_var.get() else "robot.urdf.xacro"
        selected_world_name = self.selected_world.get()
        # Find full path of selected world
        world_file = next((f for f in self.world_files if os.path.basename(f) == selected_world_name), None)
        if not world_file:
            self.status_label.configure(text="Error: No world file selected", text_color="#E74C3C")
            return
        
        # Set spawn coordinates based on world
        spawn_x = "0.0"
        spawn_y = "0.0"
        if "igvc.sdf" in selected_world_name:
            spawn_x = "11.0"
            spawn_y = "0"

        # Build then launch as requested
        cmd = (f"cd {self.workspace_root} && "
               f"colcon build --packages-select esda_simulation_2025 && "
               f"ros2 launch esda_simulation_2025 launch_sim.launch.py use_lidar:={lidar} world_file:={world_file} spawn_x:={spawn_x} spawn_y:={spawn_y} robot_model:={robot_model}")
        self.run_in_terminal("SIM", cmd)

    def toggle_slam(self):
        if not self.is_sim_running():
            self.status_label.configure(text="Error: Launch Simulation first!", text_color="#E74C3C")
            return
        
        selected_costmap_name = self.selected_costmap.get()
        scan_topic = "/scan_fused" if self.lane_detection_var.get() else "/scan"
        
        # Check if user wants to load an existing map
        if selected_costmap_name != "[New Costmap]":
            costmap_file = next((f for f in self.costmap_files if os.path.basename(f) == selected_costmap_name), None)
            if not costmap_file:
                self.status_label.configure(text="Error: Costmap file not found", text_color="#E74C3C")
                return
            # slam_toolbox expects map_file_name WITHOUT extension (.yaml, .pgm)
            # and looks for .data and .posegraph files (serialized SLAM map format)
            # Remove the .yaml extension from the path
            map_file_base = costmap_file.rsplit('.', 1)[0]
            
            # Check if SLAM serialized map files exist (.data and .posegraph)
            if os.path.exists(f"{map_file_base}.data") and os.path.exists(f"{map_file_base}.posegraph"):
                # Launch SLAM with preloaded map in mapping mode (allows adding to existing map)
                # map_start_at_dock tells slam_toolbox to load and continue from the saved map
                cmd = (f"ros2 launch esda_simulation_2025 online_async_launch.py "
                       f"use_sim_time:=true "
                       f"map_file_name:={map_file_base} "
                       f"map_start_at_dock:=true "
                       f"scan_topic:={scan_topic}")
                self.status_label.configure(text=f"Loading SLAM map: {selected_costmap_name}...", text_color="#F1C40F")
            else:
                # Serialized SLAM map doesn't exist - this is likely a Nav2/AMCL map only
                self.status_label.configure(text=f"Note: '{selected_costmap_name}' has no SLAM data. Starting new SLAM map...", text_color="#F39C12")
                # Launch SLAM in new mapping mode
                cmd = (f"ros2 launch esda_simulation_2025 online_async_launch.py "
                       f"use_sim_time:=true "
                       f"scan_topic:={scan_topic}")
        else:
            # Launch SLAM in mapping mode (create new map)
            cmd = (f"ros2 launch esda_simulation_2025 online_async_launch.py "
                   f"use_sim_time:=true "
                   f"scan_topic:={scan_topic}")
        
        self.status_label.configure(text="Waiting for simulation to stabilize...", text_color="#F1C40F")
        self.update()
        threading.Thread(target=self._launch_slam_delayed, args=(cmd,), daemon=True).start()
    
    def _launch_slam_delayed(self, cmd):
        time.sleep(3)  # Wait for simulation to be ready
        self.run_in_terminal("SLAM", cmd)

    def toggle_lane_detection(self):
        """Launch (or stop, if already running) the lane detection node standalone.

        Independent of SLAM/AMCL/Nav2 - the "Enable Lane Detection" checkbox still
        controls whether those modules are pointed at /scan_fused vs /scan, but
        actually starting the detector is a separate, explicit action here.
        """
        if not self.is_sim_running():
            self.status_label.configure(text="Error: Launch Simulation first!", text_color="#E74C3C")
            return

        mode = self.lane_detector_mode.get()

        show_visualization = (
            "true"
            if self.lane_visualization.get() == "Visualization On"
            else "false"
        )

        if mode == "FCN":
            model_path = f"{self.workspace_root}/lane-detection-on-rural-roads-master/CS542_Project/Code/FCN_model.h5"
            lane_cmd = (
                f"ros2 run esda_simulation_2025 lane_detection_FCN.py "
                f"--ros-args "
                f"-p fcn_model_path:={model_path} "
                f"-p show_visualization:={show_visualization}"
            )
        elif mode == "TwinLiteNet+":
            repo_path = f"{self.workspace_root}/TwinLiteNetPlus"
            weight_path = f"{repo_path}/pretrained/nano.pth"
            lane_cmd = (
                f"ros2 run esda_simulation_2025 lane_detection_twinlite.py "
                f"--ros-args -p twinlite_repo_path:={repo_path} "
                f"-p twinlite_weight_path:={weight_path} "
                f"-p twinlite_variant:=nano"
                f"-p show_visualization:={show_visualization}"
            )
        else:
            lane_cmd = (
                f"ros2 run esda_simulation_2025 lane_detection.py "
                f"--ros-args "
                f"-p show_visualization:={show_visualization}"
            )

        self.run_in_terminal("LANE", lane_cmd)

        # Side cameras have no stereo pair - remap left/right/depth to the same
        # monocular feed and its depth sensor, and remap outputs so they don't
        # collide with the front camera's /lane_markers, /lane_obstacles, /scan_fused.
        if self.dual_camera_var.get():
            for side in ("left_camera", "right_camera"):
                side_lane_cmd = (
                    f"ros2 run esda_simulation_2025 lane_detection.py "
                    f"--ros-args "
                    f"-r __node:=lane_detection_{side} "
                    f"-r /camera/left/image_raw:=/camera/{side}/image_raw "
                    f"-r /camera/right/image_raw:=/camera/{side}/image_raw "
                    f"-r /camera/depth/image_raw:=/camera/{side}/depth/image_raw "
                    f"-r /lane_markers:=/lane_markers_{side} "
                    f"-r /lane_obstacles:=/lane_obstacles_{side} "
                    f"-r /scan_fused:=/scan_fused_{side} "
                    f"-p camera_frame_id:={side}_link_optical "
                    f"-p camera_mount_height:=0.3 "
                    f"-p camera_mount_pitch:=-0.1 "
                    f"-p max_lane_range:=3.0 "
                    f"-p show_visualization:={show_visualization}"
                )
                self.run_in_terminal(f"LANE_{side.upper()}", side_lane_cmd)

    def toggle_amcl(self):
        if not self.is_sim_running():
            self.status_label.configure(text="Error: Launch Simulation first!", text_color="#E74C3C")
            return
        selected_costmap_name = self.selected_costmap.get()
        if selected_costmap_name == "[New Costmap]":
            self.status_label.configure(text="Use SLAM for new costmap creation", text_color="#F39C12")
            return
        # Find full path of selected costmap
        costmap_file = next((f for f in self.costmap_files if os.path.basename(f) == selected_costmap_name), None)
        if not costmap_file:
            self.status_label.configure(text="Error: Costmap file not found", text_color="#E74C3C")
            return
        self.status_label.configure(text="Waiting for simulation to stabilize...", text_color="#F1C40F")
        self.update()
        threading.Thread(target=self._launch_amcl_delayed, args=(costmap_file,), daemon=True).start()
    
    def _launch_amcl_delayed(self, costmap_file):
        time.sleep(3)  # Wait for simulation to be ready
        scan_topic = "/scan_fused" if self.lane_detection_var.get() else "/scan"
        cmd = (f"ros2 launch esda_simulation_2025 localization_launch.py "
               f"use_sim_time:=true map:={costmap_file} "
               f"amcl_base_frame_id:=base_link amcl_odom_frame_id:=odom "
               f"scan_topic:={scan_topic}")
        self.run_in_terminal("AMCL", cmd)

    def toggle_nav(self):
        if not self.is_sim_running():
            self.status_label.configure(text="Error: Launch Simulation first!", text_color="#E74C3C")
            return
        selected_costmap_name = self.selected_costmap.get()
        scan_topic = "/scan_fused" if self.lane_detection_var.get() else "/scan"
        
        if selected_costmap_name == "[New Costmap]":
            # Launch Nav2 without a map (for SLAM mode)
            cmd = (f"ros2 launch esda_simulation_2025 navigation_launch.py use_sim_time:=true "
                   f"map_subscribe_transient_local:=true "
                   f"scan_topic:={scan_topic}")
        else:
            # Find full path of selected costmap
            costmap_file = next((f for f in self.costmap_files if os.path.basename(f) == selected_costmap_name), None)
            if not costmap_file:
                self.status_label.configure(text="Error: Costmap file not found", text_color="#E74C3C")
                return
            cmd = (f"ros2 launch esda_simulation_2025 navigation_launch.py use_sim_time:=true "
                   f"map_subscribe_transient_local:=true "
                   f"map:={costmap_file} "
                   f"scan_topic:={scan_topic}")
        self.status_label.configure(text="Waiting for localization to be ready...", text_color="#F1C40F")
        self.update()
        threading.Thread(target=self._launch_nav_delayed, args=(cmd,), daemon=True).start()
    
    def _launch_nav_delayed(self, cmd):
        time.sleep(2)  # Wait for AMCL/SLAM to be ready
        self.run_in_terminal("NAV", cmd)

    def toggle_rviz(self):
        rviz_config = f"{self.workspace_root}/src/esda_simulation_2025/config/view_bot.rviz"
        # Add a small delay if SLAM was just launched to ensure map is published
        if "SLAM" in self.processes and self.processes["SLAM"].poll() is None:
            self.status_label.configure(text="Waiting for SLAM to publish map...", text_color="#F1C40F")
            self.update()
            threading.Thread(target=self._launch_rviz_delayed, args=(rviz_config,), daemon=True).start()
        else:
            use_sim_time = "true" if self.is_sim_running() else "false"
            cmd = f"rviz2 -d {rviz_config} --ros-args -p use_sim_time:={use_sim_time}"
            self.run_in_terminal("RVIZ", cmd)

    def toggle_follow_the_gap(self):
        if not self.is_sim_running():
            self.status_label.configure(text="Error: Launch Simulation first!", text_color="#E74C3C")
            return
        cmd = f"ros2 run esda_simulation_2025 follow_the_gap.py"
        self.run_in_terminal("FOLLOW_THE_GAP", cmd)

    def launch_track_follower(self):
        if not self.is_sim_running():
            self.status_label.configure(text="Error: Launch Simulation first!", text_color="#E74C3C")
            return
        cmd = f"ros2 run esda_simulation_2025 track_follower.py"
        self.run_in_terminal("TRACK_FOLLOWER", cmd)

    def launch_behaviour_tree(self):
        if not self.is_sim_running():
            self.status_label.configure(text="Error: Launch Simulation first!", text_color="#E74C3C")
            return
        cmd = f"ros2 run esda_simulation_2025 behaviour_tree.py"
        self.run_in_terminal("BEHAVIOUR_TREE", cmd)
    
    def _launch_rviz_delayed(self, rviz_config):
        time.sleep(2)  # Wait for SLAM to publish the map
        cmd = f"rviz2 -d {rviz_config} --ros-args -p use_sim_time:=true"
        self.run_in_terminal("RVIZ", cmd)

    def toggle_teleop(self):
        # We need to run telemetry inside the script location
        cmd = f"python3 {self.workspace_root}/src/esda_simulation_2025/scripts/teleop_wasd.py"
        self.run_in_terminal("TELEOP", cmd)

    def kill_all(self):
        self.status_label.configure(text="Cleaning up Gazebo and processes...", text_color="#E74C3C")
        self.update()
        # Kill all Gazebo/Ignition instances aggressively
        subprocess.run("killall -9 gzserver gzclient gazebo ruby gz ign ign-gazebo-server 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f 'gz sim' 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f ign 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f gazebo 2>/dev/null", shell=True)
        # Kill lane detection processes by command-line pattern, not just self.processes.
        # If the UI was restarted while one was running, it's no longer tracked in
        # self.processes but the xterm/bash/ros2 node tree is still alive and eating
        # CPU -- this matches on all three (xterm's -e argument embeds the same
        # command string), so it kills the whole tree regardless of tracking state.
        subprocess.run("pkill -9 -f lane_detection.py 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f lane_detection_FCN.py 2>/dev/null", shell=True)
        subprocess.run("pkill -9 -f lane_detection_twinlite.py 2>/dev/null", shell=True)
        # Clean up shared memory segments that often cause FastDDS errors
        subprocess.run("rm -rf /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null", shell=True)
        # Kill our managed processes
        for name in list(self.processes.keys()):
            self.stop_process(name)
        time.sleep(0.5)  # Give processes time to terminate
        self.status_label.configure(text="System Reset", text_color="#BDC3C7")
    
    def is_sim_running(self):
        """Check if simulation is currently running"""
        return "SIM" in self.processes and self.processes["SIM"].poll() is None
    
    def check_clock(self):
        """Check if /clock topic is publishing (diagnostics)"""
        self.status_label.configure(text="Checking /clock topic...", text_color="#F1C40F")
        self.update()
        
        # Try to find ROS_DISTRO or default to humble
        ros_distro = os.environ.get("ROS_DISTRO", "humble")
        ros_setup = f"/opt/ros/{ros_distro}/setup.bash"
        if not os.path.exists(ros_setup):
            ros_setup_cmd = "source /opt/ros/*/setup.bash 2>/dev/null || true"
        else:
            ros_setup_cmd = f"source {ros_setup}"

        cmd = (f'xterm -T "Clock Diagnostics" -geometry 80x20 -e "bash -c \\"'
               f'{ros_setup_cmd} && '
               f'echo \\"Checking /clock topic (simulation time)...\\" && '
               f'echo \\"If you see data, simulation time is working.\\" && '
               f'echo \\"If timeout, check if simulation is running.\\" && '
               f'echo && '
               f'ros2 topic echo /clock --once; '
               f'echo && echo \\"Press Enter to close...\\" && read\\""')
        
        subprocess.Popen(cmd, shell=True)
        self.status_label.configure(text="Diagnostics window opened", text_color="#3498DB")
    
    def launch_waypoint_navigator(self):
        """Launch the waypoint navigator UI"""
        self.status_label.configure(text="Launching Waypoint Navigator...", text_color="#F1C40F")
        self.update()
        
        cmd = f"python3 {self.workspace_root}/src/esda_simulation_2025/scripts/waypoint_navigator.py"
        
        try:
            subprocess.Popen(cmd, shell=True)
            self.status_label.configure(text="Waypoint Navigator launched", text_color="#2ECC71")
        except Exception as e:
            self.status_label.configure(text=f"Error launching: {str(e)}", text_color="#E74C3C")

    # EKF now launches automatically as part of launch_sim.launch.py.

if __name__ == "__main__":
    app = SimManager()
    app.mainloop()
