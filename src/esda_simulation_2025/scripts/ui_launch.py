import customtkinter as ctk # type: ignore
import subprocess
import os
import signal
import time
import glob
import threading

class SimManager(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("ESDA Simulation Manager")
        self.geometry("700x540")
        

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
        self.workspace_root = "/esda_sim_ws"
        
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
        self.label = ctk.CTkLabel(self, text="ESDA Simulation Suite", font=("Orbitron", 24, "bold"), text_color=self.accent_blue, bg_color=self.bg_dark)
        self.label.grid(row=0, column=0, pady=12)

        # Build Section
        self.build_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.build_frame.grid(row=1, column=0, pady=6, padx=12, sticky="ew")
        
        self.build_button = ctk.CTkButton(self.build_frame, text="1. Colcon Build (Required for changes)", 
                          command=self.build_workspace, 
                          fg_color=self.accent_green, hover_color="#1E8449", font=("Orbitron", 14, "bold"), text_color=self.bg_dark)
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
                            width=340, fg_color=self.bg_dark, button_color=self.accent_blue, text_color=self.fg_text)
        self.world_dropdown.grid(row=0, column=1, padx=6, pady=3, sticky="ew", columnspan=2)
        
        # Costmap File Selection
        self.costmap_label = ctk.CTkLabel(self.file_frame, text="Costmap YAML:", font=("Orbitron", 11), text_color=self.accent_purple, bg_color=self.bg_panel)
        self.costmap_label.grid(row=1, column=0, padx=6, pady=3, sticky="w")
        
        costmap_options = ["[New Costmap]"] + [os.path.basename(f) for f in self.costmap_files]
        self.costmap_dropdown = ctk.CTkOptionMenu(self.file_frame, variable=self.selected_costmap,
                              values=costmap_options,
                              width=340, fg_color=self.bg_dark, button_color=self.accent_blue, text_color=self.fg_text)
        self.costmap_dropdown.grid(row=1, column=1, padx=6, pady=3, sticky="ew", columnspan=2)


        # Simulation Section (LIDAR and Lane Detection side by side)
        self.sim_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.sim_frame.grid(row=3, column=0, pady=6, padx=16, sticky="ew")
        self.sim_frame.grid_columnconfigure(0, weight=1)
        self.sim_frame.grid_columnconfigure(1, weight=1)

        self.lidar_var = ctk.BooleanVar(value=True)
        self.lidar_check = ctk.CTkCheckBox(self.sim_frame, text="Enable LIDAR", variable=self.lidar_var, font=("Orbitron", 14), text_color=self.accent_blue, bg_color=self.bg_panel)
        self.lidar_check.grid(row=0, column=0, padx=10, pady=6, sticky="w")

        self.lane_detection_var = ctk.BooleanVar(value=True)
        self.lane_detection_check = ctk.CTkCheckBox(self.sim_frame, text="Enable Lane Detection", variable=self.lane_detection_var, font=("Orbitron", 14), text_color=self.accent_purple, bg_color=self.bg_panel)
        self.lane_detection_check.grid(row=0, column=1, padx=10, pady=6, sticky="w")

        self.sim_button = ctk.CTkButton(self.sim_frame, text="2. Launch Simulation", command=self.toggle_sim, font=("Orbitron", 16, "bold"), fg_color=self.accent_blue, hover_color="#1E90FF", text_color=self.bg_dark)
        self.sim_button.grid(row=0, column=2, padx=10, pady=6, sticky="e")

        # Remove SLAM Options Section (now merged)

        # Modules Label
        self.mod_label = ctk.CTkLabel(self, text="Navigation & SLAM Modules", font=("Orbitron", 13, "italic"), text_color=self.accent_blue, bg_color=self.bg_dark)
        self.mod_label.grid(row=5, column=0, pady=(8, 2))

        # Modules Section
        self.modules_frame = ctk.CTkFrame(self, fg_color=self.bg_panel)
        self.modules_frame.grid(row=6, column=0, pady=6, padx=12, sticky="ew")
        
        self.slam_button = ctk.CTkButton(self.modules_frame, text="Launch SLAM", command=self.toggle_slam, font=("Orbitron", 12), fg_color=self.accent_purple, hover_color="#5F27CD", text_color=self.bg_dark)
        self.slam_button.grid(row=0, column=0, padx=6, pady=6, sticky="ew")
        self.amcl_button = ctk.CTkButton(self.modules_frame, text="Launch AMCL", command=self.toggle_amcl, font=("Orbitron", 12), fg_color=self.accent_blue, hover_color="#1E90FF", text_color=self.bg_dark)
        self.amcl_button.grid(row=0, column=1, padx=6, pady=6, sticky="ew")
        self.nav_button = ctk.CTkButton(self.modules_frame, text="Launch Nav2", command=self.toggle_nav, font=("Orbitron", 12), fg_color=self.accent_green, hover_color="#00B894", text_color=self.bg_dark)
        self.nav_button.grid(row=1, column=0, padx=6, pady=6, sticky="ew")
        self.rviz_button = ctk.CTkButton(self.modules_frame, text="Launch RViz2", command=self.toggle_rviz, font=("Orbitron", 12), fg_color=self.accent_orange, hover_color="#FFB300", text_color=self.bg_dark)
        self.rviz_button.grid(row=1, column=1, padx=6, pady=6, sticky="ew")
        self.modules_frame.grid_columnconfigure((0,1), weight=1)

        # Teleop Section
        self.teleop_button = ctk.CTkButton(self, text="Launch WASD Teleop Terminal", command=self.toggle_teleop,
                           fg_color=self.accent_orange, hover_color="#FFB300", font=("Orbitron", 14, "bold"), text_color=self.bg_dark)
        self.teleop_button.grid(row=7, column=0, pady=6, padx=12, sticky="ew")
        
        # Diagnostics Section
        self.diag_button = ctk.CTkButton(self, text="Check /clock Topic (Diagnostics)", command=self.check_clock,
                         fg_color=self.accent_blue, hover_color="#1E90FF", font=("Orbitron", 11), text_color=self.bg_dark)
        self.diag_button.grid(row=8, column=0, pady=3, padx=12, sticky="ew")

        # Emergency Section
        self.kill_button = ctk.CTkButton(self, text="KILL ALL PROCESSES & RESET GAZEBO", command=self.kill_all, 
                         fg_color=self.accent_red, hover_color="#7B241C", font=("Orbitron", 12, "bold"), text_color=self.bg_dark)
        self.kill_button.grid(row=9, column=0, pady=10, padx=12, sticky="ew")

        self.status_label = ctk.CTkLabel(self, text="System Ready", text_color=self.fg_dim, font=("Orbitron", 10), bg_color=self.bg_dark)
        self.status_label.grid(row=10, column=0, pady=4)

        # Store original colors
        self.default_colors = {
            "SIM": self.sim_button.cget("fg_color"),
            "SLAM": self.slam_button.cget("fg_color"),
            "AMCL": self.amcl_button.cget("fg_color"),
            "NAV": self.nav_button.cget("fg_color"),
            "RVIZ": self.rviz_button.cget("fg_color"),
            "TELEOP": self.teleop_button.cget("fg_color"),
            "LANE": self.slam_button.cget("fg_color")
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

    def run_in_terminal(self, name, command):
        if name in self.processes and self.processes[name].poll() is None:
            self.stop_process(name)
            return

        # Prepare the command to source ROS and our workspace
        # We also disable SHM transport to avoid FastDDS errors in Docker
        full_command = (f'xterm -T "{name}" -geometry 100x30 -e "bash -c \\"'
                        f'export FASTRTPS_DEFAULT_PROFILES_FILE={self.workspace_root}/src/esda_simulation_2025/config/fastdds_noshm.xml && '
                        f'source /opt/ros/humble/setup.bash && '
                        f'source {self.workspace_root}/install/setup.bash && '
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
        elif name == "LANE":
            # Lane detection doesn't have its own button, just update status
            pass

    def build_workspace(self):
        self.status_label.configure(text="Building... Check terminal window", text_color="#F1C40F")
        self.update()
        
        cmd = f'xterm -T "Build Process" -e "bash -c \\"cd {self.workspace_root} && colcon build --packages-select esda_simulation_2025; echo; echo Done. Press Enter to close.; read\\""'
        proc = subprocess.run(cmd, shell=True)
        
        self.status_label.configure(text="Build attempt finished", text_color="#BDC3C7")

    def toggle_sim(self):
        lidar = "true" if self.lidar_var.get() else "false"
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
               f"ros2 launch esda_simulation_2025 launch_sim.launch.py use_lidar:={lidar} world_file:={world_file} spawn_x:={spawn_x} spawn_y:={spawn_y}")
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
            # Remove the .yaml extension from the path
            map_file_base = costmap_file.rsplit('.', 1)[0]
            # Launch SLAM with preloaded map (localization mode with map updates)
            cmd = (f"ros2 launch esda_simulation_2025 online_async_launch.py "
                   f"use_sim_time:=true "
                   f"map_file_name:={map_file_base} "
                   f"mode:=localization "
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
        
        # Launch lane detection if enabled
        if self.lane_detection_var.get():
            time.sleep(2)  # Give SLAM time to start
            lane_cmd = (f"ros2 run esda_simulation_2025 lane_detection.py ")
            self.run_in_terminal("LANE", lane_cmd)

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
        cmd = f"rviz2 -d {rviz_config} --ros-args -p use_sim_time:=true"
        self.run_in_terminal("RVIZ", cmd)

    def toggle_teleop(self):
        # We need to run telemetry inside the script location
        cmd = f"python3 {self.workspace_root}/src/esda_simulation_2025/scripts/teleop_wasd.py"
        self.run_in_terminal("TELEOP", cmd)

    def kill_all(self):
        self.status_label.configure(text="Cleaning up Gazebo and processes...", text_color="#E74C3C")
        self.update()
        # Kill processes and clean up shared memory segments that often cause FastDDS errors
        subprocess.run("pkill -f ign && pkill -f gz && rm -rf /dev/shm/fastrtps_*", shell=True)
        for name in list(self.processes.keys()):
            self.stop_process(name)
        self.status_label.configure(text="System Reset", text_color="#BDC3C7")
    
    def is_sim_running(self):
        """Check if simulation is currently running"""
        return "SIM" in self.processes and self.processes["SIM"].poll() is None
    
    def check_clock(self):
        """Check if /clock topic is publishing (diagnostics)"""
        self.status_label.configure(text="Checking /clock topic...", text_color="#F1C40F")
        self.update()
        
        cmd = (f'xterm -T "Clock Diagnostics" -geometry 80x20 -e "bash -c \\"'
               f'source /opt/ros/humble/setup.bash && '
               f'echo \\"Checking /clock topic (simulation time)...\\" && '
               f'echo \\"If you see data, simulation time is working.\\" && '
               f'echo \\"If timeout, check if simulation is running.\\" && '
               f'echo && '
               f'ros2 topic echo /clock --once; '
               f'echo && echo \\"Press Enter to close...\\" && read\\""')
        
        subprocess.Popen(cmd, shell=True)
        self.status_label.configure(text="Diagnostics window opened", text_color="#3498DB")

if __name__ == "__main__":
    app = SimManager()
    app.mainloop()
