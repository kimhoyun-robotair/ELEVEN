#!/usr/bin/env python3
"""Command a pivot turn and evaluate measured odometry and wheel motion."""
import argparse
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

LEFT = ("front_left_wheel_joint", "rear_left_wheel_joint")
RIGHT = ("front_right_wheel_joint", "rear_right_wheel_joint")


def yaw_from_quaternion(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def wrapped_delta(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PivotTest(Node):
    def __init__(self):
        super().__init__("scout_twin_pivot_test")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscription = self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, qos)
        self.joint_subscription = self.create_subscription(JointState, "/joint_states", self.on_joints, qos)
        self.pose = None
        self.previous_yaw = None
        self.unwrapped_yaw = 0.0
        self.sim_time = None
        self.latest_wall = None
        self.odom_samples = 0
        self.initial_xy = None
        self.peak_displacement = 0.0
        self.collect_wheels = False
        self.left_speeds, self.right_speeds = [], []

    def on_odom(self, msg):
        if msg.header.frame_id != "sim_world" or msg.child_frame_id != "base_link":
            return
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise RuntimeError("Odometry contains a nonfinite pose value.")
        if abs(sum(v*v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 0.02:
            raise RuntimeError("Odometry quaternion is not normalized.")
        yaw = yaw_from_quaternion(q)
        if self.previous_yaw is not None:
            self.unwrapped_yaw += wrapped_delta(yaw - self.previous_yaw)
        self.previous_yaw = yaw
        self.pose = (p.x, p.y)
        if self.initial_xy is not None:
            self.peak_displacement = max(self.peak_displacement, math.dist(self.pose, self.initial_xy))
        self.sim_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.latest_wall = time.monotonic()
        self.odom_samples += 1

    def on_joints(self, msg):
        if not self.collect_wheels or len(msg.name) != len(msg.velocity):
            return
        velocities = dict(zip(msg.name, msg.velocity))
        if not all(name in velocities for name in LEFT + RIGHT):
            return
        if not all(math.isfinite(velocities[name]) for name in LEFT + RIGHT):
            raise RuntimeError("Wheel velocities contain NaN or infinity.")
        self.left_speeds.append(sum(velocities[name] for name in LEFT) / 2)
        self.right_speeds.append(sum(velocities[name] for name in RIGHT) / 2)

    def command(self, omega=0.0):
        command = Twist()
        command.angular.z = omega
        self.publisher.publish(command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angle-deg", type=float, default=90.0)
    parser.add_argument("--speed", type=float, default=0.35, help="Angular speed magnitude, rad/s")
    parser.add_argument("--max-displacement", type=float, default=0.30, help="Maximum XY drift in metres")
    parser.add_argument("--yaw-tolerance-deg", type=float, default=12.0)
    parser.add_argument("--wall-timeout", type=float, default=120.0)
    parser.add_argument("--output", type=Path, help="Optional JSON result path")
    args = parser.parse_args()
    if not 10.0 <= abs(args.angle_deg) <= 360.0:
        parser.error("absolute angle must be between 10 and 360 degrees")
    if not 0.1 <= args.speed <= 0.6:
        parser.error("speed must be between 0.1 and 0.6 rad/s")
    if args.max_displacement <= 0 or args.wall_timeout < 10 or args.yaw_tolerance_deg <= 0:
        parser.error("thresholds must be positive and wall timeout at least 10 seconds")
    os.environ.setdefault("ROS_DOMAIN_ID", "73")
    rclpy.init()
    node = PivotTest()
    errors = []
    report = {"passed": False, "commanded_angle_deg": args.angle_deg}
    target = math.radians(abs(args.angle_deg))
    sign = 1.0 if args.angle_deg > 0 else -1.0
    start_yaw = None
    try:
        print("Pivot test: waiting for live odometry and /cmd_vel subscription...", flush=True)
        ready_deadline = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.odom_samples >= 3 and node.publisher.get_subscription_count() > 0:
                break
        if node.odom_samples < 3 or node.publisher.get_subscription_count() == 0:
            raise RuntimeError("Live /ground_truth/odom or /cmd_vel subscriber unavailable; start Isaac Sim and press Play.")

        # Let the command settle and confirm advancing timestamps before moving.
        settle_start = node.sim_time
        settle_wall = time.monotonic() + min(30.0, args.wall_timeout)
        while rclpy.ok() and node.sim_time - settle_start < 0.5:
            if time.monotonic() > settle_wall:
                raise RuntimeError("Simulation clock did not advance while waiting to settle.")
            node.command()
            rclpy.spin_once(node, timeout_sec=0.05)

        start_yaw = node.unwrapped_yaw
        start_sim = node.sim_time
        node.initial_xy = node.pose
        initial_pose = node.pose
        deadline = time.monotonic() + args.wall_timeout
        # Skid slip makes measured body yaw slower than ideal wheel kinematics.
        sim_timeout = max(20.0, 6.0 * target / args.speed + 10.0)
        print(f"Commanding {args.angle_deg:.1f} deg pivot; linear velocity = 0.", flush=True)
        next_command = 0.0
        while rclpy.ok():
            now = time.monotonic()
            elapsed = node.sim_time - start_sim
            signed_yaw = sign * (node.unwrapped_yaw - start_yaw)
            if signed_yaw >= target:
                break
            if now > deadline:
                raise RuntimeError("Wall timeout before reaching target angle.")
            if elapsed > sim_timeout:
                raise RuntimeError("Simulation timeout; pivot did not reach the target angle.")
            if elapsed < -1e-6:
                raise RuntimeError("Simulation was reset during the test.")
            if now - node.latest_wall > 3.0:
                raise RuntimeError("Odometry stopped for more than 3 wall seconds.")
            if signed_yaw < -math.radians(15):
                raise RuntimeError("Yaw turned opposite to the requested direction.")
            if node.peak_displacement > args.max_displacement:
                raise RuntimeError("XY displacement exceeded the configured pivot tolerance.")
            node.collect_wheels = elapsed > 0.5 and signed_yaw < 0.9 * target
            if now >= next_command:
                # Approach at a lower speed to avoid overshoot, still zero linear speed.
                omega = max(min(0.25, args.speed), min(args.speed, 1.3 * max(0.0, target - signed_yaw)))
                node.command(sign * omega)
                next_command = now + 0.05
            rclpy.spin_once(node, timeout_sec=0.02)

        node.collect_wheels = False
        stop_sim = node.sim_time
        stop_wall = time.monotonic() + 20.0
        while rclpy.ok() and node.sim_time - stop_sim < 1.0:
            if time.monotonic() > stop_wall:
                errors.append("Simulation did not advance during final settling.")
                break
            node.command()
            rclpy.spin_once(node, timeout_sec=0.05)

        measured_angle = math.degrees(node.unwrapped_yaw - start_yaw)
        left_mean = sum(node.left_speeds) / len(node.left_speeds) if node.left_speeds else None
        right_mean = sum(node.right_speeds) / len(node.right_speeds) if node.right_speeds else None
        if abs(measured_angle - args.angle_deg) > args.yaw_tolerance_deg:
            errors.append("Measured final yaw is outside the configured tolerance.")
        if node.peak_displacement > args.max_displacement:
            errors.append("Peak XY displacement exceeds the configured pivot tolerance.")
        if len(node.left_speeds) < 3 or len(node.right_speeds) < 3:
            errors.append("Too few wheel velocity samples to confirm counter-rotation.")
        elif not (sign * left_mean < -0.05 and sign * right_mean > 0.05):
            errors.append("Wheel velocities do not confirm left/right counter-rotation.")
        report.update({
            "measured_angle_deg": round(measured_angle, 3),
            "final_xy_displacement_m": round(math.dist(node.pose, initial_pose), 4),
            "peak_xy_displacement_m": round(node.peak_displacement, 4),
            "left_mean_rad_s": left_mean, "right_mean_rad_s": right_mean,
            "wheel_samples": len(node.left_speeds),
            "elapsed_simulation_s": node.sim_time - start_sim,
            "passed": not errors,
        })
    except (RuntimeError, KeyboardInterrupt) as exc:
        errors.append(str(exc) or "Interrupted by user.")
        if start_yaw is not None:
            report["measured_angle_deg"] = math.degrees(node.unwrapped_yaw - start_yaw)
            report["peak_xy_displacement_m"] = node.peak_displacement
    finally:
        node.collect_wheels = False
        # Repeated zero commands allow DDS to deliver the stop before shutdown.
        for _ in range(5):
            if rclpy.ok():
                node.command()
                rclpy.spin_once(node, timeout_sec=0.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report["errors"] = errors
    result = json.dumps(report, indent=2)
    print(result)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
