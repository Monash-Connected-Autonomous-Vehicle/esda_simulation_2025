#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import select
import sys
import termios
import time
import tty

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/s : forward/backward
a/d : left/right
x   : stop

Hold a key to keep moving; the robot stops when you let go.

Speed control (RHS):
u/j : increase/decrease max linear speed by 10%
i/k : increase/decrease max angular speed by 10%

CTRL-C to quit
"""

move_bindings = {
    'w': (1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (-1, 0),
    'x': (0, 0),
}

speed_bindings = {
    'u': (1.2, 1.0),
    'j': (0.8, 1.0),
    'i': (1.0, 1.2),
    'k': (1.0, 0.8),
}

# /cmd_vel is re-published at PUBLISH_PERIOD while a key is held, so the
# ESP32 bridge / diff drive controller (0.5 s cmd_vel timeout) never sees a
# gap. HOLD_TIME must be longer than the keyboard's auto-repeat delay
# (500 ms here), otherwise the first press stutters before repeats begin.
PUBLISH_PERIOD = 0.1
HOLD_TIME = 0.6

def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('teleop_wasd')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    

    speed = 2
    turn = 2
    x = 0.0
    th = 0.0
    last_key_time = 0.0
    held = False

    try:
        print(msg)
        while True:
            key = get_key(settings, PUBLISH_PERIOD)
            if not key:
                if not held:
                    continue
                if time.monotonic() - last_key_time > HOLD_TIME:
                    # Key released: stop once, then go quiet.
                    held = False
                    x = 0.0
                    th = 0.0
            elif key in move_bindings.keys():
                x = move_bindings[key][0]
                th = move_bindings[key][1]
            elif key in speed_bindings.keys():
                speed = speed * speed_bindings[key][0]
                turn = turn * speed_bindings[key][1]
                print(f"Currently: speed {speed:.2f}\tturn {turn:.2f}")
            elif key == '\x03':  # CTRL-C
                break
            else:
                x = 0.0
                th = 0.0

            if key:
                last_key_time = time.monotonic()
                held = True

            twist = Twist()
            twist.linear.x = float(x * speed)
            twist.angular.z = float(th * turn)
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
