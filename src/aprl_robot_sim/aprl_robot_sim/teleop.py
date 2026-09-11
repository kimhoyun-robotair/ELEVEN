"""Terminal Twist publisher with a wall-clock key-repeat deadman."""

import argparse
import math
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


HELP = """
AMR keyboard control (focus this terminal)
  w / s       forward / reverse
  a / d       rotate left / right
  u / o       forward-left / forward-right arc
  Space / x   stop immediately
  + / -       linear speed +/- 0.05 m/s (0.05 .. 2.0)
  ] / [       angular speed +/- 0.1 rad/s (0.1 .. 1.5)
  q / Ctrl-C  stop and exit
Motion lasts only while motion key events arrive. The terminal repeat delay
may cause a brief initial stop. Releasing a key stops after --deadman seconds.
"""


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--turn", type=float, default=0.5)
    parser.add_argument("--deadman", type=float, default=0.25)
    parsed = parser.parse_args(remove_ros_args(args=args)[1:])
    if not (math.isfinite(parsed.speed) and 0.05 <= parsed.speed <= 2.0):
        parser.error("--speed must be between 0.05 and 2.0 m/s")
    if not (math.isfinite(parsed.turn) and 0.1 <= parsed.turn <= 1.5):
        parser.error("--turn must be between 0.1 and 1.5 rad/s")
    if not (math.isfinite(parsed.deadman) and 0.05 <= parsed.deadman <= 0.5):
        parser.error("--deadman must be between 0.05 and 0.5 wall seconds")
    if not sys.stdin.isatty():
        parser.error("teleop needs an interactive terminal (TTY)")
    rclpy.init(args=args)
    node = Node("amr_keyboard_teleop")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    original_settings = termios.tcgetattr(sys.stdin)
    speed, turn = parsed.speed, parsed.turn
    v, w, last_motion_key = 0.0, 0.0, -math.inf
    moves = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1),
             "u": (1, 1), "o": (1, -1)}
    print(HELP, flush=True)
    print(f"linear={speed:.2f} m/s angular={turn:.2f} rad/s; ROS_DOMAIN_ID must match simulator.", flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.04)[0] else ""
            if key in ("q", "\x03"):
                break
            if key in moves:
                multiplier_v, multiplier_w = moves[key]
                v, w = multiplier_v * speed, multiplier_w * turn
                last_motion_key = time.monotonic()
            elif key in (" ", "x"):
                v, w, last_motion_key = 0.0, 0.0, -math.inf
            elif key in ("+", "=", "-", "[", "]"):
                if key in ("+", "=", "-"):
                    speed = min(2.0, max(0.05, speed + (-0.05 if key == "-" else 0.05)))
                else:
                    turn = min(1.5, max(0.1, turn + (-0.1 if key == "[" else 0.1)))
                print(f"linear={speed:.2f} m/s angular={turn:.2f} rad/s", flush=True)
            if time.monotonic() - last_motion_key > parsed.deadman:
                v, w = 0.0, 0.0
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        if rclpy.ok():
            for _ in range(5):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
