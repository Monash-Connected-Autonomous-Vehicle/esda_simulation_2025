#!/usr/bin/env python3
"""
ESP32 wheel bridge for hardware-in-the-loop testing.

Talks to the two wheel_controller_left / wheel_controller_right ESP32 boards
over USB serial (same line protocol as two_wheel_gui.py) and:

  /cmd_vel (Twist)  ->  differential drive kinematics  ->  "VELOCITY <rpm>"

It also serves a small web GUI (default http://127.0.0.1:8767) for driving
the robot by linear/angular velocity or by setting each wheel's RPM directly.

Publishes:
  /esp32/left/rpm, /esp32/right/rpm   (Float32, measured wheel RPM)
  /esp32/joint_states                 (JointState, measured wheel positions)

Safety:
  - The firmware has no command watchdog, so this node sends STOP whenever
    commands stop arriving for longer than cmd_vel_timeout, and on shutdown.
  - Wheel RPMs are clamped to max_forward_rpm / max_reverse_rpm. When one
    wheel saturates, both are scaled together so the turn radius is kept.
"""

import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

import serial
from serial.tools import list_ports

BAUD = 115200
WHEELS = ('left', 'right')

# Used only when the board's READY line was missed (e.g. native USB boards
# that don't reset when the port is opened). fwdMaxSafeOffsetUs differs
# between the two firmware builds, so the CONFIG reply identifies the side.
CONFIG_FINGERPRINT = {'155': 'left', '80': 'right'}


class WheelLink:
    """Serial connection to one wheel's ESP32."""

    def __init__(self, name, logger):
        self.name = name
        self.logger = logger
        self.connection = None
        self.port = ''
        self.write_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.samples = deque(maxlen=400)
        self.events = deque(maxlen=40)
        self.config = {}
        self.board_reset = False  # set when READY is seen; bridge resends target
        self.running = True
        self.reader = None

    @property
    def connected(self):
        return bool(self.connection and self.connection.is_open)

    def attach(self, connection):
        self.connection = connection
        self.port = connection.port
        self.add_event(f'Connected to {self.port}')
        self.logger.info(f'{self.name} wheel connected on {self.port}')
        self.reader = threading.Thread(target=self._reader, daemon=True)
        self.reader.start()

    def disconnect(self):
        with self.write_lock:
            if self.connection:
                try:
                    self.connection.close()
                except (serial.SerialException, OSError):
                    pass
            self.connection = None
            self.port = ''

    def send(self, command):
        with self.write_lock:
            if not self.connected:
                return False
            try:
                self.connection.write((command.strip() + '\n').encode('ascii'))
            except (serial.SerialException, OSError) as error:
                self.add_event(f'TX FAILED: {error}')
                self.logger.error(f'{self.name} write failed: {error}')
                self.connection.close()
                self.connection = None
                self.port = ''
                return False
        self.add_event(f'> {command.strip()}')
        return True

    def add_event(self, message):
        with self.data_lock:
            self.events.append({'time': time.strftime('%H:%M:%S'), 'message': message})

    def latest(self):
        with self.data_lock:
            return self.samples[-1] if self.samples else None

    def _reader(self):
        connection = self.connection
        while self.running and connection is self.connection and connection.is_open:
            try:
                raw_line = connection.readline()
            except (serial.SerialException, OSError, TypeError) as error:
                self.add_event(f'RX FAILED: {error}')
                self.logger.error(f'{self.name} serial lost: {error}')
                self.disconnect()
                return
            line = raw_line.decode('utf-8', errors='replace').strip()
            if line:
                self.handle_line(line)

    def handle_line(self, line):
        fields = line.split(',')
        if fields[0] == 'DATA' and len(fields) >= 10:
            try:
                sample = {
                    'time': int(fields[1]),
                    'mode': int(fields[2]),
                    'target': float(fields[3]),
                    'wheel_rpm': float(fields[4]),
                    'motor_rpm': float(fields[5]),
                    'pwm': int(fields[6]),
                    'ticks': int(fields[7]),
                    'wheel_revs': float(fields[8]),
                    'error': float(fields[9]),
                    'stamp': time.monotonic(),
                }
            except ValueError:
                self.add_event(f'Bad telemetry: {line}')
                return
            with self.data_lock:
                self.samples.append(sample)
        elif fields[0] == 'CONFIG':
            self.config = {item.split(':', 1)[0]: item.split(':', 1)[1]
                           for item in fields[1:] if ':' in item}
        elif fields[0] == 'READY':
            self.board_reset = True
            self.add_event(line)
        elif fields[0] == 'EVENT':
            self.add_event(fields[1] if len(fields) > 1 else line)
        elif fields[0] in ('WARNING', 'ACK', 'ERROR'):
            self.add_event(line)

    def state(self):
        with self.data_lock:
            samples = list(self.samples)[-200:]
            events = list(self.events)
        return {
            'connected': self.connected,
            'port': self.port,
            'sample': samples[-1] if samples else {},
            'samples': [{'time': s['time'], 'target': s['target'], 'wheel_rpm': s['wheel_rpm']}
                        for s in samples],
            'events': events,
        }


def identify_board(port, wait_s=2.5):
    """Open a port and work out which wheel firmware is on it.

    Returns (side, serial.Serial) or (None, None). Looks for the READY line
    first (boards that reset on open), then falls back to asking for CONFIG.
    """
    try:
        connection = serial.Serial(port, BAUD, timeout=0.1)
    except (serial.SerialException, OSError):
        return None, None

    deadline = time.monotonic() + wait_s
    asked_config = False
    while time.monotonic() < deadline:
        try:
            line = connection.readline().decode('utf-8', errors='replace').strip()
        except (serial.SerialException, OSError):
            break
        if line.startswith('READY,wheel_controller_'):
            side = line.split('_')[-1]
            if side in WHEELS:
                return side, connection
        if line.startswith('CONFIG,'):
            for item in line.split(',')[1:]:
                key, _, value = item.partition(':')
                if key == 'fwdMaxSafeOffsetUs' and value in CONFIG_FINGERPRINT:
                    return CONFIG_FINGERPRINT[value], connection
        if not asked_config and time.monotonic() > deadline - wait_s / 2:
            try:
                connection.write(b'CONFIG\n')
            except (serial.SerialException, OSError):
                break
            asked_config = True

    connection.close()
    return None, None


class Esp32WheelBridge(Node):

    def __init__(self):
        super().__init__('esp32_wheel_bridge')

        self.declare_parameter('left_port', 'auto')
        self.declare_parameter('right_port', 'auto')
        self.declare_parameter('wheel_radius', 0.1625)      # m, robot_core_ref.xacro
        self.declare_parameter('wheel_separation', 0.5)     # m, my_controllers_2wd.yaml
        self.declare_parameter('max_forward_rpm', 35.0)     # Right board tops out ~36 RPM
        self.declare_parameter('max_reverse_rpm', 13.0)     # Right board's only reverse plateau
        self.declare_parameter('left_invert', False)
        self.declare_parameter('right_invert', False)
        self.declare_parameter('cmd_vel_timeout', 0.5)      # s, 0 disables
        self.declare_parameter('rpm_deadband', 0.5)         # don't resend smaller changes
        self.declare_parameter('gui_host', '127.0.0.1')
        self.declare_parameter('gui_port', 8767)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')

        p = lambda name: self.get_parameter(name).value
        self.port_params = {'left': p('left_port'), 'right': p('right_port')}
        self.wheel_radius = float(p('wheel_radius'))
        self.wheel_separation = float(p('wheel_separation'))
        self.max_forward_rpm = float(p('max_forward_rpm'))
        self.max_reverse_rpm = float(p('max_reverse_rpm'))
        self.invert = {'left': bool(p('left_invert')), 'right': bool(p('right_invert'))}
        self.cmd_vel_timeout = float(p('cmd_vel_timeout'))
        self.rpm_deadband = float(p('rpm_deadband'))
        self.joint_names = {'left': p('left_joint'), 'right': p('right_joint')}

        self.links = {name: WheelLink(name, self.get_logger()) for name in WHEELS}

        # Latest command, from /cmd_vel or the GUI's direct RPM mode.
        self.command_lock = threading.Lock()
        self.command_source = 'none'           # 'cmd_vel' | 'rpm' | 'none'
        self.command_twist = (0.0, 0.0)
        self.command_rpm = {'left': 0.0, 'right': 0.0}
        self.command_stamp = 0.0
        self.desired_rpm = {'left': 0.0, 'right': 0.0}
        self.saturated = False
        self.last_sent = {'left': None, 'right': None}
        self.last_resend = {'left': 0.0, 'right': 0.0}

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.rpm_pubs = {name: self.create_publisher(Float32, f'/esp32/{name}/rpm', 10)
                         for name in WHEELS}
        self.joint_pub = self.create_publisher(JointState, '/esp32/joint_states', 10)

        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.05, self.publish_telemetry)

        self.connect_thread = threading.Thread(target=self.connect_loop, daemon=True)
        self.connect_thread.start()

        self.gui = ThreadingHTTPServer((p('gui_host'), int(p('gui_port'))), make_handler(self))
        threading.Thread(target=self.gui.serve_forever, daemon=True).start()
        self.get_logger().info(
            f'ESP32 wheel bridge started. Web GUI at http://{p("gui_host")}:{p("gui_port")}')

    # ------------------------------------------------------------
    # Serial connection management
    # ------------------------------------------------------------

    def connect_loop(self):
        warned = False
        while rclpy.ok():
            missing = [name for name in WHEELS if not self.links[name].connected]
            if missing:
                self.try_connect(missing)
                still_missing = [n for n in WHEELS if not self.links[n].connected]
                if still_missing and not warned:
                    self.get_logger().warn(
                        f'Waiting for ESP32 on: {", ".join(still_missing)} '
                        f'(ports seen: {[p.device for p in list_ports.comports()]})')
                    warned = True
                elif not still_missing:
                    warned = False
            time.sleep(2.0)

    def try_connect(self, missing):
        in_use = {link.port for link in self.links.values() if link.connected}

        # Explicitly configured ports first.
        for name in missing:
            port = self.port_params[name]
            if port and port != 'auto' and port not in in_use:
                try:
                    connection = serial.Serial(port, BAUD, timeout=0.1)
                except (serial.SerialException, OSError):
                    continue
                self.last_sent[name] = None
                self.links[name].attach(connection)
                in_use.add(port)

        auto = [n for n in missing if self.port_params[n] in ('', 'auto')
                and not self.links[n].connected]
        if not auto:
            return
        explicit = {self.port_params[n] for n in WHEELS}
        candidates = [p.device for p in list_ports.comports()
                      if p.device not in in_use and p.device not in explicit
                      and ('ttyACM' in p.device or 'ttyUSB' in p.device)]
        for port in candidates:
            side, connection = identify_board(port)
            if side is None:
                continue
            if side in auto and not self.links[side].connected:
                self.last_sent[side] = None
                self.links[side].attach(connection)
                auto.remove(side)
            else:
                connection.close()
            if not auto:
                return

    # ------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------

    def cmd_vel_callback(self, msg):
        with self.command_lock:
            self.command_source = 'cmd_vel'
            self.command_twist = (msg.linear.x, msg.angular.z)
            self.command_stamp = time.monotonic()

    def set_direct_rpm(self, left, right):
        with self.command_lock:
            self.command_source = 'rpm'
            self.command_rpm = {'left': float(left), 'right': float(right)}
            self.command_stamp = time.monotonic()

    def publish_gui_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)  # cmd_vel_callback picks it up

    def stop(self):
        with self.command_lock:
            self.command_source = 'none'
            self.command_stamp = 0.0
        # Also zero /cmd_vel so cmd_vel_odometry stops integrating.
        self.cmd_vel_pub.publish(Twist())
        for name in WHEELS:
            self.send_target(name, 0.0, force=True)

    def twist_to_rpm(self, linear, angular):
        v_left = linear - angular * self.wheel_separation / 2.0
        v_right = linear + angular * self.wheel_separation / 2.0
        to_rpm = 60.0 / (2.0 * math.pi * self.wheel_radius)
        return {'left': v_left * to_rpm, 'right': v_right * to_rpm}

    def limit(self, rpm):
        """Scale both wheels together so neither exceeds its limit."""
        scale = 1.0
        for value in rpm.values():
            cap = self.max_forward_rpm if value >= 0.0 else self.max_reverse_rpm
            if abs(value) > cap > 0.0:
                scale = min(scale, cap / abs(value))
        return {name: value * scale for name, value in rpm.items()}, scale < 1.0

    def control_loop(self):
        now = time.monotonic()
        with self.command_lock:
            source = self.command_source
            stamp = self.command_stamp
            twist = self.command_twist
            direct = dict(self.command_rpm)

        if source != 'none' and self.cmd_vel_timeout > 0.0 and now - stamp > self.cmd_vel_timeout:
            source = 'none'

        if source == 'cmd_vel':
            rpm = self.twist_to_rpm(*twist)
        elif source == 'rpm':
            rpm = direct
        else:
            rpm = {'left': 0.0, 'right': 0.0}

        rpm, self.saturated = self.limit(rpm)
        self.desired_rpm = rpm
        for name in WHEELS:
            self.send_target(name, rpm[name])

    def send_target(self, name, rpm, force=False):
        link = self.links[name]
        if not link.connected:
            return
        if abs(rpm) < 0.05:
            rpm = 0.0

        # Board rebooted, or it dropped out of VELOCITY mode on its own:
        # forget what we last sent so the target goes out again.
        latest = link.latest()
        stalled = (latest is not None and rpm != 0.0 and latest['mode'] == 0
                   and time.monotonic() - self.last_resend[name] > 1.0)
        if link.board_reset or stalled:
            link.board_reset = False
            self.last_sent[name] = None

        last = self.last_sent[name]
        if not force and last is not None:
            if rpm == 0.0 and last == 0.0:
                return
            # Each VELOCITY resets the firmware PID, so only send real changes.
            if rpm != 0.0 and abs(rpm - last) < self.rpm_deadband:
                return

        wire_rpm = -rpm if self.invert[name] else rpm
        command = 'STOP' if rpm == 0.0 else f'VELOCITY {wire_rpm:.2f}'
        if link.send(command):
            self.last_sent[name] = rpm
            self.last_resend[name] = time.monotonic()

    # ------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------

    def publish_telemetry(self):
        joint = JointState()
        joint.header.stamp = self.get_clock().now().to_msg()
        for name in WHEELS:
            sample = self.links[name].latest()
            if sample is None or not self.links[name].connected:
                continue
            sign = -1.0 if self.invert[name] else 1.0
            rpm = sign * sample['wheel_rpm']
            self.rpm_pubs[name].publish(Float32(data=float(rpm)))
            joint.name.append(self.joint_names[name])
            joint.position.append(sign * sample['wheel_revs'] * 2.0 * math.pi)
            joint.velocity.append(rpm * 2.0 * math.pi / 60.0)
        if joint.name:
            self.joint_pub.publish(joint)

    def gui_state(self):
        with self.command_lock:
            source = self.command_source
            twist = self.command_twist
            stamp = self.command_stamp
        if self.cmd_vel_timeout > 0.0 and time.monotonic() - stamp > self.cmd_vel_timeout:
            source = 'none'
        return {
            'source': source,
            'twist': {'linear': twist[0], 'angular': twist[1]},
            'desired': self.desired_rpm,
            'saturated': self.saturated,
            'limits': {'forward': self.max_forward_rpm, 'reverse': self.max_reverse_rpm},
            'timeout': self.cmd_vel_timeout,
            'wheels': {name: self.links[name].state() for name in WHEELS},
        }

    def shutdown(self):
        self.gui.shutdown()
        for link in self.links.values():
            link.running = False
            link.send('STOP')
            link.disconnect()


def make_handler(bridge):

    class Handler(BaseHTTPRequestHandler):

        def _json(self, payload, status=200):
            body = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/':
                body = HTML.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/api/state':
                self._json(bridge.gui_state())
            else:
                self.send_error(404)

        def do_POST(self):
            length = int(self.headers.get('Content-Length', '0'))
            try:
                data = json.loads(self.rfile.read(length) or b'{}')
                if self.path == '/api/twist':
                    bridge.publish_gui_twist(float(data['linear']), float(data['angular']))
                elif self.path == '/api/rpm':
                    bridge.set_direct_rpm(float(data['left']), float(data['right']))
                elif self.path == '/api/stop':
                    bridge.stop()
                else:
                    self.send_error(404)
                    return
                self._json({'ok': True})
            except (KeyError, ValueError, TypeError) as error:
                self._json({'error': str(error)}, 400)

        def log_message(self, *_args):
            pass

    return Handler


HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESDA Wheel Drive</title>
<style>
:root{--ink:#182126;--muted:#647277;--paper:#f4f0e8;--panel:#fffdf8;--line:#d9d4c8;--orange:#d95f32;--teal:#237f78;--blue:#3569a8;--red:#b43e36}*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:15px Georgia,serif}header{padding:22px clamp(16px,5vw,64px) 14px;border-bottom:1px solid var(--line)}h1{font-size:clamp(26px,4vw,40px);line-height:.95;margin:0}h1 span{color:var(--orange)}header p{margin:6px 0 0;color:var(--muted)}.layout{display:flex;flex-direction:column;gap:18px;padding:18px clamp(16px,5vw,64px)}.row2{display:grid;grid-template-columns:1fr 1fr;gap:18px}section{background:var(--panel);border:1px solid var(--line);padding:16px;border-radius:6px}h2{font-size:17px;margin:0 0 10px}label{display:flex;justify-content:space-between;color:var(--muted);font:12px ui-monospace,monospace;text-transform:uppercase;letter-spacing:.05em;margin:12px 0 4px}input,button{font:inherit;border:1px solid var(--line);border-radius:4px;padding:8px;background:#fff;color:var(--ink)}input[type=number]{width:100%}input[type=range]{width:100%;padding:0}button{cursor:pointer;font-weight:bold}button.primary{background:var(--teal);color:#fff;border-color:var(--teal)}button.danger{background:var(--red);color:#fff;border-color:var(--red)}button.active{outline:3px solid var(--orange)}.actions{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}.status{padding:8px;background:#ece8df;border-left:4px solid var(--muted);font:12px ui-monospace,monospace;margin-bottom:8px}.status.ok{border-color:var(--teal)}.status.bad{border-color:var(--red)}.status.warn{border-color:var(--orange)}table{width:100%;border-collapse:collapse;font:13px ui-monospace,monospace}td,th{padding:6px 4px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child{text-align:left}th{color:var(--muted);font-weight:normal;text-transform:uppercase;font-size:11px}canvas{width:100%;height:240px;border:1px solid var(--line);background:#fff}.log{height:140px;overflow:auto;background:#20282b;color:#d9e8df;padding:9px;font:12px ui-monospace,monospace;white-space:pre-wrap}.hint{color:var(--muted);font-size:13px;line-height:1.35}.legend{display:flex;flex-wrap:wrap;gap:16px;font:12px ui-monospace,monospace;margin-top:6px}.swatch{width:12px;height:12px;border-radius:2px;display:inline-block;margin-right:5px;vertical-align:middle}.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;max-width:260px;margin-top:10px}.pad button{padding:12px 0}
@media(max-width:800px){.row2{grid-template-columns:1fr}}
</style></head>
<body><header><h1>ESDA Wheel <span>Drive</span></h1><p>Drive both ESP32 wheel controllers through ROS. Commands stop automatically if this page stops sending.</p></header>
<main class="layout">
<div class="row2">
<section><h2>Left wheel</h2><div id="status_left" class="status bad">Disconnected</div></section>
<section><h2>Right wheel</h2><div id="status_right" class="status bad">Disconnected</div></section>
</div>
<button class="danger" style="width:100%;padding:16px;font-size:18px" onclick="stopAll()">STOP (Space)</button>
<div class="row2">
<section><h2>Drive by velocity (/cmd_vel)</h2>
<label><span>Linear x, m/s</span><span id="lin_val">0.00</span></label><input id="lin" type="range" min="-0.25" max="0.6" step="0.01" value="0" oninput="sliderChanged()">
<label><span>Angular z, rad/s</span><span id="ang_val">0.00</span></label><input id="ang" type="range" min="-1.5" max="1.5" step="0.05" value="0" oninput="sliderChanged()">
<div class="actions"><button id="twistBtn" class="primary" onclick="startTwist()">Drive</button><button onclick="zeroSliders()">Zero sliders</button></div>
<div class="pad"><span></span><button onmousedown="nudge(1,0)">W</button><span></span><button onmousedown="nudge(0,1)">A</button><button onmousedown="nudge(0,0)">X</button><button onmousedown="nudge(0,-1)">D</button><span></span><button onmousedown="nudge(-1,0)">S</button><span></span></div>
<p class="hint">Publishes on /cmd_vel at 10 Hz while driving, so RViz odometry follows. Keys W/A/S/D/X also work.</p>
</section>
<section><h2>Drive by wheel RPM (direct)</h2>
<label><span>Left wheel RPM</span></label><input id="rpm_l" type="number" step="0.5" value="15">
<label><span>Right wheel RPM</span></label><input id="rpm_r" type="number" step="0.5" value="15">
<div class="actions"><button id="rpmBtn" class="primary" onclick="startRpm()">Run</button><button onclick="$('rpm_r').value=$('rpm_l').value">Copy L to R</button></div>
<p class="hint" id="limits"></p>
</section>
</div>
<section><h2>Live</h2><div id="cmdStatus" class="status">Idle</div>
<table><thead><tr><th>Metric</th><th>Left</th><th>Right</th></tr></thead><tbody>
<tr><td>Bridge target RPM</td><td id="des_left">0</td><td id="des_right">0</td></tr>
<tr><td>Firmware target RPM</td><td id="target_left">0</td><td id="target_right">0</td></tr>
<tr><td>Measured RPM</td><td id="rpm_left">0</td><td id="rpm_right">0</td></tr>
<tr><td>PWM (us)</td><td id="pwm_left">1500</td><td id="pwm_right">1500</td></tr>
<tr><td>Encoder ticks</td><td id="ticks_left">0</td><td id="ticks_right">0</td></tr>
</tbody></table>
<canvas id="plot"></canvas>
<div class="legend"><span><i class="swatch" style="background:#237f78"></i>Left RPM</span><span><i class="swatch" style="background:#d95f32"></i>Right RPM</span><span><i class="swatch" style="background:#3569a8"></i>Left target</span><span><i class="swatch" style="background:#b43e36"></i>Right target</span></div>
</section>
<section><h2>Serial events</h2><div id="log" class="log"></div></section>
</main>
<script>
const $=id=>document.getElementById(id);
let mode='none',timer=null,state=null;
async function post(url,body){let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});let d=await r.json();if(!r.ok||d.error)throw new Error(d.error||r.status);return d}
function setMode(m){mode=m;if(timer){clearInterval(timer);timer=null}$('twistBtn').classList.toggle('active',m==='twist');$('rpmBtn').classList.toggle('active',m==='rpm');
 if(m==='twist')timer=setInterval(sendTwist,100);if(m==='rpm')timer=setInterval(sendRpm,100)}
function sendTwist(){post('/api/twist',{linear:+$('lin').value,angular:+$('ang').value}).catch(e=>status('Send failed: '+e.message,'bad'))}
function sendRpm(){post('/api/rpm',{left:+$('rpm_l').value,right:+$('rpm_r').value}).catch(e=>status('Send failed: '+e.message,'bad'))}
function startTwist(){setMode('twist');sendTwist()}
function startRpm(){setMode('rpm');sendRpm()}
async function stopAll(){setMode('none');try{await post('/api/stop')}catch(e){status('STOP failed: '+e.message,'bad')}}
function sliderChanged(){$('lin_val').textContent=(+$('lin').value).toFixed(2);$('ang_val').textContent=(+$('ang').value).toFixed(2)}
function zeroSliders(){$('lin').value=0;$('ang').value=0;sliderChanged()}
function nudge(l,a){$('lin').value=l*0.3;$('ang').value=a*0.8;sliderChanged();if(mode!=='twist')startTwist()}
function status(m,c){$('cmdStatus').textContent=m;$('cmdStatus').className='status '+(c||'')}
document.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT'&&e.target.type==='number')return;const k=e.key.toLowerCase();
 if(k===' '){e.preventDefault();stopAll()}else if(k==='w')nudge(1,0);else if(k==='s')nudge(-1,0);else if(k==='a')nudge(0,1);else if(k==='d')nudge(0,-1);else if(k==='x')nudge(0,0)});
function draw(){let c=$('plot'),x=c.getContext('2d');let w=c.width=c.clientWidth*devicePixelRatio,h=c.height=c.clientHeight*devicePixelRatio;x.clearRect(0,0,w,h);if(!state)return;
 let dl=state.wheels.left.samples,dr=state.wheels.right.samples;let n=Math.max(dl.length,dr.length);if(!n)return;let vals=[0,1];for(let s of dl.concat(dr))vals.push(s.wheel_rpm,s.target);
 let min=Math.min(...vals),max=Math.max(...vals);let px=i=>i/(n-1||1)*w,py=v=>h-(v-min)/(max-min||1)*h;
 x.strokeStyle='#d9d4c8';x.lineWidth=devicePixelRatio;x.beginPath();x.moveTo(0,py(0));x.lineTo(w,py(0));x.stroke();
 function line(s,k,col){if(!s.length)return;x.strokeStyle=col;x.lineWidth=2*devicePixelRatio;x.beginPath();s.forEach((p,i)=>i?x.lineTo(px(i),py(p[k])):x.moveTo(px(i),py(p[k])));x.stroke()}
 line(dl,'target','#3569a8');line(dr,'target','#b43e36');line(dl,'wheel_rpm','#237f78');line(dr,'wheel_rpm','#d95f32')}
async function poll(){try{let r=await fetch('/api/state');state=await r.json()}catch(e){status('Bridge unreachable: '+e.message,'bad');return}
 let lines=[];for(let w of ['left','right']){let s=state.wheels[w],p=s.sample||{};$('status_'+w).textContent=s.connected?'Connected: '+s.port:'Disconnected (bridge keeps retrying)';$('status_'+w).className='status '+(s.connected?'ok':'bad');
  $('des_'+w).textContent=state.desired[w].toFixed(2);$('target_'+w).textContent=(p.target||0).toFixed(2);$('rpm_'+w).textContent=(p.wheel_rpm||0).toFixed(2);$('pwm_'+w).textContent=p.pwm||1500;$('ticks_'+w).textContent=p.ticks||0;
  for(let e of s.events)lines.push(e.time+' ['+w[0].toUpperCase()+'] '+e.message)}
 lines.sort();let log=$('log');let atBottom=log.scrollTop+log.clientHeight>=log.scrollHeight-4;log.textContent=lines.join('\n');if(atBottom)log.scrollTop=log.scrollHeight;
 $('limits').textContent='Limits: +'+state.limits.forward+' / -'+state.limits.reverse+' RPM. Both wheels are scaled together if either exceeds its limit. Commands time out after '+state.timeout+' s.';
 let src=state.source==='cmd_vel'?'/cmd_vel  v='+state.twist.linear.toFixed(2)+' m/s  w='+state.twist.angular.toFixed(2)+' rad/s':state.source==='rpm'?'Direct RPM':'Stopped';
 status('Source: '+src+(state.saturated?'  (SATURATED, scaled down)':''),state.saturated?'warn':(state.source==='none'?'':'ok'));draw()}
setInterval(poll,250);poll();sliderChanged();
</script></body></html>'''


def main(args=None):
    rclpy.init(args=args)
    node = Esp32WheelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
