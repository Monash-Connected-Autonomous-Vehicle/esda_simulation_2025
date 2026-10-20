#!/usr/bin/env python3
"""
ESDA Sim Control GUI  —  one window to run / kill the whole sim stack (WSL).

Run it from a ROS-sourced terminal so the drive pad works:
    sudo apt install -y python3-tk            # one-time, if missing
    source /opt/ros/humble/setup.bash
    source ~/esda_ws/install/setup.bash
    python3 /mnt/c/work/felix/mcav/for_server/esda_gui.py

Launch buttons start each piece in the background (logs in ~/esda_logs/).
STOP ALL kills everything. The arrow pad drives via /cmd_vel (hold a button).
"""
import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

WS = os.path.expanduser("~/esda_ws")
LOGS = os.path.expanduser("~/esda_logs")
# Where this checkout lives inside WSL. The old hard-coded
# /mnt/c/uni/2026_sem1/mcav/esda path no longer exists. Override with the
# ESDA_SRC environment variable if you move the repo.
ESDA_SRC = os.environ.get("ESDA_SRC", "/mnt/c/work/felix/mcav/for_server")
CAPTURE = os.path.join(ESDA_SRC, "capture_frames.py")
VISION = os.path.join(ESDA_SRC, "vision", "lane_seg_node.py")
RVIZ_CFG = f"{WS}/install/esda_simulation_2025/share/esda_simulation_2025/config/view_bot.rviz"
os.makedirs(LOGS, exist_ok=True)

# every launched process gets software rendering + ROS sourced
ENV = (f"export LIBGL_ALWAYS_SOFTWARE=1; "
       f"source /opt/ros/humble/setup.bash; source {WS}/install/setup.bash; ")

LIN = 0.3   # drive speeds
ANG = 0.6

# Fallback patterns for things started OUTSIDE this GUI. Every entry is a full
# invocation, never a bare word. The previous list contained bare words like
# "nav2", "create", "ruby" and "amcl"; `pkill -9 -f nav2` kills an editor that
# merely has nav2_params.yaml open, and "create" matches almost anything.
# Processes the GUI itself started are killed by process group instead - see
# stop_all() below.
STOP_PATTERNS = [
    "ros2 launch esda_simulation_2025",
    "ros2 run teleop_twist_keyboard",
    "vision/lane_seg_node.py",
    "capture_frames.py",
    "ign gazebo",
    "rviz2 -d",
    "ros_gz_bridge parameter_bridge",
    "async_slam_toolbox_node",
]

# ---- optional ROS drive pad (works if rclpy is importable) ----
try:
    import rclpy
    from geometry_msgs.msg import Twist
    HAVE_ROS = True
except Exception:
    HAVE_ROS = False


class Driver:
    def __init__(self):
        self.lin = 0.0
        self.ang = 0.0
        self.ok = HAVE_ROS
        if HAVE_ROS:
            try:
                rclpy.init()
                self.node = rclpy.create_node("esda_gui_drive")
                self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
                threading.Thread(target=self._loop, daemon=True).start()
            except Exception:
                self.ok = False

    def set(self, lin, ang):
        self.lin, self.ang = lin, ang

    def _loop(self):
        while rclpy.ok():
            t = Twist()
            t.linear.x = float(self.lin)
            t.angular.z = float(self.ang)
            try:
                self.pub.publish(t)
            except Exception:
                pass
            time.sleep(0.1)


class App:
    def __init__(self, root):
        self.root = root
        self.driver = Driver()
        root.title("ESDA Sim Control")
        root.configure(bg="#1e1e1e")

        def log(msg):
            self.status.configure(state="normal")
            self.status.insert("end", msg + "\n")
            self.status.see("end")
            self.status.configure(state="disabled")

        self.log = log

        children = {}          # name -> Popen, each its own process group

        def launch(name, cmd):
            logf = open(f"{LOGS}/{name}.log", "w")
            # start_new_session=True puts the child in its own process group,
            # so stop_all() can kill it and all of ITS children precisely,
            # without pattern-matching anything.
            p = subprocess.Popen(["bash", "-lc", ENV + cmd], stdout=logf,
                                 stderr=subprocess.STDOUT, start_new_session=True)
            children[name] = p
            log(f"started {name}  (pgid {p.pid}, log: ~/esda_logs/{name}.log)")

        def stop_all():
            # 1. everything this GUI started, by process group
            killed = []
            for name, p in list(children.items()):
                if p.poll() is not None:
                    children.pop(name, None)
                    continue
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except OSError:
                    pass
                killed.append(name)
            time.sleep(1.5)
            for name, p in list(children.items()):
                if p.poll() is None:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except OSError:
                        pass
                children.pop(name, None)
            # 2. narrow fallback for anything started in a terminal
            for pat in STOP_PATTERNS:
                subprocess.run(["pkill", "-f", pat],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            log("STOP ALL — stopped: " + (", ".join(killed) if killed else "(none of ours)"))

        # ---- launch buttons ----
        top = tk.Frame(root, bg="#1e1e1e")
        top.pack(padx=12, pady=10, fill="x")
        tk.Label(top, text="Run the stack", fg="#9cdcfe", bg="#1e1e1e",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        btns = [
            ("1  Launch SIM",  lambda: launch("sim",  "ros2 launch esda_simulation_2025 launch_sim_headless.launch.py"), "#2d7d46"),
            ("2  SLAM (map)",  lambda: launch("slam", "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true"), "#2d7d46"),
            ("3  Nav2 (auto)", lambda: launch("nav2", "ros2 launch esda_simulation_2025 navigation_launch.py use_sim_time:=true"), "#2d7d46"),
            ("RViz",           lambda: launch("rviz", f"rviz2 -d {RVIZ_CFG}"), "#3a6ea5"),
            ("Gazebo GUI",     lambda: launch("gzgui", "ign gazebo -g"), "#3a6ea5"),
            ("Capture frames", lambda: launch("capture", f"python3 {CAPTURE} /head_camera/images 1.0 ~/lane_dataset"), "#3a6ea5"),
            ("Lane vision",    lambda: launch("vision", f"python3 {VISION}"), "#8e44ad"),
        ]
        for i, (label, cmd, color) in enumerate(btns):
            b = tk.Button(top, text=label, command=cmd, width=18, bg=color, fg="white",
                          relief="flat", font=("Segoe UI", 10, "bold"), activebackground=color)
            b.grid(row=1 + i // 2, column=i % 2, padx=4, pady=4, sticky="we")

        tk.Button(top, text="■  STOP ALL", command=stop_all, bg="#c0392b", fg="white",
                  relief="flat", font=("Segoe UI", 10, "bold"), activebackground="#c0392b"
                  ).grid(row=5, column=0, columnspan=2, padx=4, pady=(8, 2), sticky="we")

        # ---- drive pad ----
        pad = tk.Frame(root, bg="#1e1e1e")
        pad.pack(padx=12, pady=8)
        tk.Label(pad, text="Drive  (hold a button)" if self.driver.ok
                 else "Drive pad needs ROS sourced (rclpy not found)",
                 fg="#9cdcfe" if self.driver.ok else "#e0a030",
                 bg="#1e1e1e", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, columnspan=3, pady=(0, 4))

        def hold(lin, ang):
            def press(_):  self.driver.set(lin, ang)
            def release(_): self.driver.set(0.0, 0.0)
            return press, release

        def padbtn(txt, r, c, lin, ang):
            b = tk.Button(pad, text=txt, width=6, height=2, bg="#333", fg="white",
                          relief="flat", font=("Segoe UI", 11, "bold"),
                          state=("normal" if self.driver.ok else "disabled"))
            p, rl = hold(lin, ang)
            b.bind("<ButtonPress-1>", p)
            b.bind("<ButtonRelease-1>", rl)
            b.grid(row=r, column=c, padx=4, pady=4)

        padbtn("▲\nFwd",  1, 1,  LIN, 0.0)
        padbtn("◀\nLeft", 2, 0,  0.0,  ANG)
        padbtn("■\nStop", 2, 1,  0.0, 0.0)
        padbtn("▶\nRight",2, 2,  0.0, -ANG)
        padbtn("▼\nBack", 3, 1, -LIN, 0.0)

        # ---- status log ----
        self.status = tk.Text(root, height=8, width=46, bg="#111", fg="#ddd",
                              relief="flat", font=("Consolas", 9), state="disabled")
        self.status.pack(padx=12, pady=(4, 12), fill="both", expand=True)
        log("ready. 1=Sim  2=SLAM  3=Nav2.  STOP ALL to clean up.")
        if not self.driver.ok:
            log("drive pad OFF — run from a ROS-sourced terminal to enable it.")


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
