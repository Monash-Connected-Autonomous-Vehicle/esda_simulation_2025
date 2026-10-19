# Achilles Hardware — Progress & Next Steps

Status log for the physical robot (code lives in `../esda-achilles-team-1/`).
Written at the point we paused hardware to document. Read this before touching
the robot — several things are non-obvious and cost us a lot of time to find.

---

## Hardware

- **Compute:** Raspberry Pi 4 (Ubuntu 22.04.5, ROS 2 Humble). Hostname `Virus26`.
- **Motor controller:** ESP32 (DOIT DevKit). Firmware: `esp32_achilles_uart/esp32_achilles_uart.ino`.
- **Motors:** 2× DC gear motors + quadrature encoders (330 counts/rev), 66 mm wheels.
- **Driver:** DRV8411 H-bridge. Enable/relay on **GPIO5**.
- **Lidar:** RPLidar (on `/dev/ttyUSB0`, CP210x USB-serial).
- **Camera:** ZED2i — used as a **plain USB camera only** (Pi has no NVIDIA GPU, so no ZED SDK / depth).
- **Pi ↔ ESP32:** GPIO UART (`/dev/serial0`), 115200 8N1.
  - ESP32 **GPIO16 = RX** ← Pi pin 8 (GPIO14/TX)
  - ESP32 **GPIO21 = TX** → Pi pin 10 (GPIO15/RX)
  - GND ↔ GND
- **Wireless E-stop:** ESP32 transmitter (`esp32_estop_transmitter/`) over ESP-NOW.

---

## What works (DONE)

### ESP32 firmware — `esp32_achilles_uart.ino` (the proven, working one)
- **Discrete commands** over UART: `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, `STOP`, `SPEED,x`, `STATUS?`.
  - **NOT** continuous velocity — there is **no `VEL` command** in this build.
- Closed-loop PID per wheel with anti-windup, per-motor PWM limits, start-up boost.
- **Spin-in-place turns** (both wheels opposite) — arc turns don't work (inner wheel stalls in the deadband).
- **Relay / driver enable on GPIO5, ACTIVE-LOW**: `relayOn()` drives it LOW = motors enabled.
- **ESP-NOW E-stop** wired to cut the relay. Master switch `ESTOP_ENABLED`.
- Telemetry back to the Pi: `TEL,vL,vR,tgtL,tgtR,pwmL,pwmR,piAlive,estop`.
- A clearly-separated `[ADDED] RASPBERRY PI UART` block feeds the same command variable as the USB serial.

### Raspberry Pi side (`esda-achilles-team-1/scripts/`)
- `keyboard_drive.py` — WASD teleop over `/dev/serial0` (exclusive lock, Ctrl+C sends STOP ×6).
- `estop.py` — **terminal emergency stop**: kills command sources + forces STOP. `alias estop='python3 ~/estop.py'`.
- `lidar_autodrive.py` — **reactive lidar autonomy**: reads `/scan`, sends FORWARD / spin-turn / STOP. Forward-only mode + turn mode with anti-chatter hysteresis. `FRONT_OFFSET=180` (robot front = −X in laser frame).
- `pi_dashboard.py` — web dashboard (sensors, E-stop, odom, one-click launches). Autostarts via `achilles-dashboard.service`.
- RPLidar bring-up over terminal → `/scan` at ~10 Hz → Foxglove (`ws://<pi>:8765`).

### Networking
- Ethernet static, **Achilles-AP** (robot's own WiFi), **iPhone "Felix" hotspot** (primary, autoconnect-priority 30), **Tailscale**.
- Connect sheet: `../esda-achilles-team-1/docs/Achilles_Connect_and_Drive.pdf`.

---

## Gotchas / hard-won lessons (READ THIS)

1. **Relay is GPIO5, ACTIVE-LOW.** LOW = motors enabled, HIGH = disabled. Driving it HIGH silently *disables* the driver (PWM goes out, nothing moves). The old code's `RELAY_PIN=26` "worked" only because it left GPIO5 untouched.
2. **Motor direction mapping (proven):** `FORWARD = both wheels NEGATIVE target`, `BACKWARD = both positive`, turns = opposite signs. The motors are mirror-ish; do **not** assume "positive = forward for both."
3. **Firmware is DISCRETE commands, not velocity.** Any Nav2/cmd_vel integration must either translate Twist → FORWARD/LEFT/RIGHT/STOP, **or** add a `VEL` command to the firmware (trade-off below).
4. **Single wheel won't spin alone; both spin fine together.** A wiring/power quirk — a single motor channel stalls even at full PWM, but both driven together run strong and balanced. Normal driving always uses both wheels, so it doesn't block driving, but it's why single-wheel bench tests were misleading.
5. **Encoders are magnitude-only** (direction inferred from the command, not quadrature). Odometry sign comes from the commanded direction.
6. **UART is single-owner.** `/dev/serial0` can only be held by one program (enforced with an exclusive lock). Run `keyboard_drive.py` **or** the bridge **or** `lidar_autodrive.py` — never two at once.
7. **ESP32 talks to the Pi over the GPIO UART, not USB.** The ESP32 USB is for flashing/monitor only. The `/dev/ttyUSB0` device is the **lidar's** CP210x, not the ESP32.
8. **No firmware command-watchdog in the uart build** — it holds the last command until STOP. Use `estop.py` / Ctrl+C as the stop. (The E-stop, when `ESTOP_ENABLED=true`, cuts power on link loss.)
9. **ZED on the Pi = plain USB camera only.** No CUDA/GPU on a Pi 4, so no ZED SDK/depth. NVIDIA Container Toolkit does **not** apply to the Pi (needs an NVIDIA GPU / Jetson).

---

## Sim → Hardware integration gaps (NEXT STEPS)

The simulation (this repo) drives on `/diff_drive_base_controller/cmd_vel_unstamped`
(continuous `Twist`) and consumes wheel odometry. The hardware firmware is discrete.
To close the loop:

1. **Decide the control interface** (key fork):
   - **(A) Keep discrete firmware** → write a bridge that maps `Twist` → FORWARD/LEFT/RIGHT/STOP. Simple, works with the proven firmware, coarse control. (`lidar_autodrive.py` already does this style.)
   - **(B) Add continuous `VEL` to the firmware** → smoother Nav2 control, but risks reintroducing the deadband/stall issues we spent a long time fixing. Needs careful low-speed PID validation.
2. **ROS → serial bridge** on the Pi:
   - subscribe `/diff_drive_base_controller/cmd_vel_unstamped`
   - convert to wheel commands, send at ~20 Hz, **send STOP when commands go stale** (don't repeat an old command forever)
   - owns `/dev/serial0` (one owner rule).
3. **Wheel odometry** — parse `TEL` telemetry → integrate → publish `/odom` (+ `odom→base_chassis_link` TF) for EKF/SLAM. (A first `esp32_bridge_node.py` exists in the hardware repo but its reader needs fixing — it wasn't parsing telemetry reliably.)
4. **`use_sim_time=false`** everywhere on hardware (several launch/config files hard-code sim time).
5. **Real sensors** — `/scan` done; add IMU; replace Gazebo sensor sources.
6. **SLAM** (slam_toolbox) on real `/scan`, then **Nav2** goals, then **autonomous slalom**.

### Known repo issues to clean up (from the hardware side)
- `slam.launch.py` launches EKF/SLAM/lane/RViz but **not** the motor connection or Nav2.
- `HARDWARE_launch_robot.launch.py` references a missing `rsp.launch.py` / `my_controllers.yaml` and outdated controller names.
- The hardware xacro selects an Arduino plugin at 57600 baud — **not** compatible with our ESP32 protocol (115200, custom).

---

## Idea for next team: a reusable bench test rig (HIL)

Because the chassis/electrical arrived late, we propose a **hardware-in-the-loop
bench rig** so software can be tested without a finished car:
- Real wheels + encoders on a stand **with a friction load** (roller/drum) so drive dynamics + odometry are realistic.
- Real lidar + movable cones; camera pointed at a **screen playing lane video** (or a fake-lane publisher).
- Same Pi + ESP32 + ROS stack; everything on standard topics (`/cmd_vel`, `/scan`, `/image`, `/odom`) + one params file.
- Foxglove/RViz for visualization.

This lets the next team test their own camera/lidar/drive code on real hardware
without the vehicle. (We have the drive stand, lidar+obstacles, and Foxglove
already — it's mostly formalizing + adding the friction load and lane screen.)

---

## File map (hardware repo: `../esda-achilles-team-1/`)

```
esp32_achilles_uart/esp32_achilles_uart.ino   <- USE THIS firmware (proven drive + Pi UART)
esp32_achilles_main/esp32_achilles_main.ino   <- older; has VEL but driving was wrong
esp32_estop_transmitter/                        <- wireless E-stop remote
esp32_motor_debug/ , esp32_motor_hold_test/     <- open-loop motor/encoder diagnostics
scripts/keyboard_drive.py                        <- WASD teleop
scripts/lidar_autodrive.py                       <- reactive lidar autonomy
scripts/estop.py                                 <- terminal E-stop
scripts/pi_dashboard.py                          <- web dashboard
src/esda_achilles_robot/                         <- ROS package (launch, config, nodes, URDF, Nav2 params)
docs/Achilles_Connect_and_Drive.pdf              <- team connect + drive quick reference
```
