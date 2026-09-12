#!/usr/bin/env python3
"""Linux terminal teleop with 20 Hz Twist publication and a key timeout."""
import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist

KEYS = {
    "i": (1.0, 0.0), ",": (-1.0, 0.0),
    "j": (0.0, 1.0), "l": (0.0, -1.0),
    "u": (1.0, 1.0), "o": (1.0, -1.0),
    "m": (-1.0, -1.0), ".": (-1.0, 1.0),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--turn", type=float, default=0.35)
    parser.add_argument("--key-timeout", type=float, default=0.5)
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("an interactive Linux terminal is required")
    if not 0 < args.speed <= 2.0 or not 0 < args.turn <= 1.5 or args.key_timeout <= 0:
        parser.error("speed must be (0, 2], turn (0, 1.5], and timeout positive")
    os.environ.setdefault("ROS_DOMAIN_ID", "73")
    rclpy.init()
    node = rclpy.create_node("scout_twin_keyboard_teleop")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    terminal = termios.tcgetattr(sys.stdin)
    speed, turn = args.speed, args.turn
    linear, angular = 0.0, 0.0
    latest_key = 0.0
    next_publish = time.monotonic()
    print("Hold/repeat i: forward, ,: reverse, j/l: pivot, k/space: stop.")
    print("u/o/m/.: arcs; q/z: increase/decrease speed; Ctrl-C: quit.")
    print(f"linear={speed:.2f} m/s, angular={turn:.2f} rad/s; key timeout={args.key_timeout:.2f}s", flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            now = time.monotonic()
            if ready:
                key = sys.stdin.read(1)
                if key == "\x03" or key == "":
                    break
                if key in KEYS:
                    linear, angular = KEYS[key]
                    latest_key = now
                elif key in ("q", "z"):
                    scale = 1.1 if key == "q" else 1.0 / 1.1
                    speed = min(2.0, max(0.02, speed * scale))
                    turn = min(1.5, max(0.05, turn * scale))
                    print(f"linear={speed:.2f} m/s, angular={turn:.2f} rad/s", flush=True)
                else:
                    linear = angular = 0.0
            if now - latest_key > args.key_timeout:
                linear = angular = 0.0
            if now >= next_publish:
                msg = Twist()
                msg.linear.x = linear * speed
                msg.angular.z = angular * turn
                publisher.publish(msg)
                next_publish = now + 0.05
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        for _ in range(5):
            if rclpy.ok():
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Stopped.")


if __name__ == "__main__":
    main()
