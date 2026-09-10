"""Live ROS stream checks; motion is an explicit --drive option."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.utilities import remove_ros_args

from .streams import StreamCollector


def pose2d(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return p.x, p.y, yaw


def checked_spin(node, wall_duration):
    end = time.monotonic() + wall_duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def command_for_sim_seconds(node, publisher, v, w, duration, wall_timeout):
    start_sim, end_wall = node.clock_seconds, time.monotonic() + wall_timeout
    if start_sim is None:
        raise RuntimeError("/clock has not arrived")
    while rclpy.ok() and node.clock_seconds - start_sim < duration:
        if time.monotonic() > end_wall:
            raise RuntimeError("simulation stopped or ran too slowly during controlled motion")
        if node.clock_seconds < start_sim:
            raise RuntimeError("simulation clock reset during motion")
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        publisher.publish(msg)
        checked_spin(node, 0.04)
    if not rclpy.ok():
        raise RuntimeError("ROS context closed during motion")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=90.0, help="wall seconds for discovery and healthy streams")
    parser.add_argument("--observe", type=float, default=5.0, help="wall seconds of continued observation after readiness")
    parser.add_argument("--drive", action="store_true", help="publish 0.15 m/s for 2 sim seconds, then stop")
    parser.add_argument("--maneuvers", action="store_true", help="also verify reverse and left/right in-place turns")
    parser.add_argument("--report", default="smoke_report.json", help="JSON report path")
    parsed = parser.parse_args(remove_ros_args(args=args)[1:])
    if not all(math.isfinite(v) and v > 0 for v in (parsed.timeout, parsed.observe)):
        parser.error("timeouts must be positive finite seconds")
    parsed.drive = parsed.drive or parsed.maneuvers
    rclpy.init(args=args)
    node = StreamCollector("amr_smoke_test")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10) if parsed.drive else None
    failures, movement = [], {"requested": parsed.drive}
    exit_code = 1
    try:
        print("Waiting for all streams, valid payloads, advancing stamps and complete TF...", flush=True)
        end = time.monotonic() + parsed.timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.problems():
                break
        failures = node.problems()
        if not failures:
            checked_spin(node, parsed.observe)
            failures = node.problems()
            for topic, item in node.snapshot().items():
                if "/depth/image_raw" in topic and item["details"].get("finite_depth_samples", 0) == 0:
                    failures.append(f"{topic}: no positive finite depth in sampled pixels")
                if topic.endswith("/scan") and item["details"].get("finite_returns", 0) == 0:
                    failures.append(f"{topic}: no finite obstacle returns in this test world")
            for side in ("front", "rear"):
                for kind in ("color", "depth"):
                    a = node.observations[f"/{side}_camera/{kind}/image_raw"].details
                    b = node.observations[f"/{side}_camera/{kind}/camera_info"].details
                    if (a.get("width"), a.get("height")) != (b.get("width"), b.get("height")):
                        failures.append(f"{side}/{kind}: image and CameraInfo dimensions differ")
        if parsed.drive and not failures:
            print("DRIVE enabled: ensure the supplied test world is running with a clear forward path.", flush=True)
            # Discovery has already completed, then settle at a commanded standstill.
            command_for_sim_seconds(node, publisher, 0.0, 0.0, 1.0, parsed.timeout)
            start = pose2d(node.latest_odom)
            command_for_sim_seconds(node, publisher, 0.15, 0.0, 2.0, parsed.timeout)
            command_for_sim_seconds(node, publisher, 0.0, 0.0, 2.0, parsed.timeout)
            finish = pose2d(node.latest_odom)
            dx, dy = finish[0]-start[0], finish[1]-start[1]
            forward = math.cos(start[2])*dx + math.sin(start[2])*dy
            lateral = -math.sin(start[2])*dx + math.cos(start[2])*dy
            yaw_change = math.atan2(math.sin(finish[2]-start[2]), math.cos(finish[2]-start[2]))
            stopped_v = abs(node.latest_odom.twist.twist.linear.x)
            stopped_w = abs(node.latest_odom.twist.twist.angular.z)
            movement.update(forward_m=forward, lateral_m=lateral, yaw_change_rad=yaw_change,
                            stopped_vx_mps=stopped_v, stopped_wz_radps=stopped_w)
            if not 0.08 <= forward <= 0.7:
                failures.append(f"drive response outside broad expected range: forward={forward:.3f} m")
            if abs(lateral) > 0.15 or abs(yaw_change) > 0.4:
                failures.append("straight command caused excessive lateral/yaw deviation")
            if stopped_v > 0.06 or stopped_w > 0.1:
                failures.append("robot did not settle after zero command")
            failures.extend(node.problems())
            if parsed.maneuvers and not failures:
                movement["maneuvers"] = {}
                for name, velocity, angular in [("reverse", -.15, 0.), ("left", 0., .4), ("right", 0., -.4)]:
                    start = pose2d(node.latest_odom)
                    command_for_sim_seconds(node, publisher, velocity, angular, 2., parsed.timeout)
                    command_for_sim_seconds(node, publisher, 0., 0., 2., parsed.timeout)
                    finish = pose2d(node.latest_odom)
                    dx, dy = finish[0]-start[0], finish[1]-start[1]
                    forward = math.cos(start[2])*dx + math.sin(start[2])*dy
                    yaw_change = math.atan2(math.sin(finish[2]-start[2]), math.cos(finish[2]-start[2]))
                    stopped_v = abs(node.latest_odom.twist.twist.linear.x)
                    stopped_w = abs(node.latest_odom.twist.twist.angular.z)
                    movement["maneuvers"][name] = {"forward_m": forward, "distance_m": math.hypot(dx,dy),
                                                   "yaw_change_rad": yaw_change, "stopped_vx_mps": stopped_v,
                                                   "stopped_wz_radps": stopped_w}
                    if name == "reverse":
                        if not -.7 <= forward <= -.08 or abs(yaw_change) > .4:
                            failures.append("reverse command did not produce stable backward travel")
                    elif not .2 <= yaw_change * (1 if angular > 0 else -1) <= 1.3 or math.hypot(dx,dy) > .15:
                        failures.append(f"{name} turn response outside expected range")
                    if stopped_v > .06 or stopped_w > .1:
                        failures.append(f"robot did not stop after {name}")
                    failures.extend(node.problems())
        if not rclpy.ok():
            failures.append("ROS context closed")
        exit_code = 1 if failures else 0
    except KeyboardInterrupt:
        failures.append("interrupted")
        exit_code = 130
    except Exception as exc:
        failures.append(f"{type(exc).__name__}: {exc}")
        exit_code = 1
    finally:
        if publisher is not None and rclpy.ok():
            for _ in range(10):
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.02)
        report = {"passed": exit_code == 0, "exit_code": exit_code,
                  "failures": sorted(set(failures)), "motion": movement,
                  "topics": node.snapshot(), "tf_edges": sorted(node.tf_edges)}
        report_path = Path(parsed.report).expanduser()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"passed": report["passed"], "report": str(report_path), "failures": report["failures"]}, indent=2), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
