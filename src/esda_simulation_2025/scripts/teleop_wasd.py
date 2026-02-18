#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
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

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
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

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key in move_bindings.keys():
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
