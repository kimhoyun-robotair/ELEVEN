#!/usr/bin/env python3
"""Observe simulation topics, payloads, stamps and TF; exit nonzero on failure."""
import argparse
import json
import math
import os
import struct
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

WHEEL_JOINTS = {
    "front_left_wheel_joint", "rear_left_wheel_joint",
    "front_right_wheel_joint", "rear_right_wheel_joint",
}


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite(values):
    return all(math.isfinite(float(v)) for v in values)


class StreamProbe(Node):
    def __init__(self, args):
        super().__init__("scout_twin_stream_probe")
        self.args = args
        self.records = {}
        self.errors = set()
        self.subscriptions_kept = []
        self.image_sizes = {}
        self.info_sizes = {}
        self.frames = {"base_link", "mid360_link", "imu_link"}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        volatile = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.add("/robot_description", String, latched, self.description, latched=True)
        self.add("/clock", Clock, volatile, self.clock)
        self.add("/joint_states", JointState, volatile, self.joints)
        self.add("/ground_truth/odom", Odometry, volatile, self.odometry)
        self.add("/mid360/imu", Imu, volatile, self.imu)
        if not args.skip_lidar:
            self.add("/mid360/points", PointCloud2, volatile, self.points)
        if not args.skip_cameras:
            for direction in ("front", "left", "right"):
                for kind in ("color", "depth"):
                    frame = f"camera_{direction}_{kind}_optical_frame"
                    self.frames.add(frame)
                    prefix = f"/camera_{direction}/{kind}"
                    self.add(prefix + "/image_raw", Image, volatile,
                             lambda msg, kind=kind, frame=frame: self.image(msg, kind, frame))
                    self.add(prefix + "/camera_info", CameraInfo, volatile,
                             lambda msg, frame=frame: self.camera_info(msg, frame))

    def add(self, topic, message_type, qos, validator, latched=False):
        self.records[topic] = {"count": 0, "first_wall": None, "last_wall": None,
                               "first_stamp": None, "last_stamp": None,
                               "stamp_advances": 0, "latched": latched}

        def receive(msg):
            record = self.records[topic]
            now = time.monotonic()
            record["count"] += 1
            record["first_wall"] = record["first_wall"] or now
            record["last_wall"] = now
            try:
                stamp = validator(msg)
                if stamp is not None:
                    if record["last_stamp"] is not None:
                        if stamp < record["last_stamp"] - 1e-9:
                            self.errors.add(f"{topic}: timestamp moved backwards (simulation reset?)")
                        if stamp > record["last_stamp"]:
                            record["stamp_advances"] += 1
                    if record["first_stamp"] is None:
                        record["first_stamp"] = stamp
                    record["last_stamp"] = stamp
            except (ValueError, AssertionError, ET.ParseError) as exc:
                self.errors.add(f"{topic}: {exc}")

        self.subscriptions_kept.append(self.create_subscription(message_type, topic, receive, qos))

    def description(self, msg):
        root = ET.fromstring(msg.data)
        assert root.tag == "robot", "robot_description is not URDF XML"
        assert WHEEL_JOINTS.issubset({j.attrib.get("name") for j in root.findall("joint")}), \
            "URDF is missing one or more expected wheel joints"
        assert root.findall("link"), "URDF has no links"

    def clock(self, msg):
        return stamp_seconds(msg.clock)

    def joints(self, msg):
        assert WHEEL_JOINTS.issubset(set(msg.name)), "expected wheel joint names are missing"
        assert len(msg.name) == len(msg.position), "position/name lengths differ"
        assert finite(msg.position), "joint position contains NaN/Inf"
        assert len(msg.velocity) in (0, len(msg.name)), "velocity/name lengths differ"
        assert finite(msg.velocity), "joint velocity contains NaN/Inf"
        return stamp_seconds(msg.header.stamp)

    def odometry(self, msg):
        assert msg.header.frame_id == "sim_world", "ground truth frame must be sim_world"
        assert msg.child_frame_id == "base_link", "odom.child_frame_id must be base_link"
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        assert finite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]), "pose contains NaN/Inf"
        assert abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 0.02, \
            "pose quaternion is not normalized"
        return stamp_seconds(msg.header.stamp)

    def imu(self, msg):
        assert msg.header.frame_id == "imu_link", "IMU frame must be imu_link"
        a, g = msg.linear_acceleration, msg.angular_velocity
        assert finite([a.x, a.y, a.z, g.x, g.y, g.z]), "IMU contains NaN/Inf"
        assert msg.orientation_covariance[0] == -1.0, \
            "six-axis IMU must mark absolute orientation unavailable (covariance[0] = -1)"
        return stamp_seconds(msg.header.stamp)

    def points(self, msg):
        assert msg.header.frame_id == "mid360_link", "cloud frame must be mid360_link"
        assert {"x", "y", "z"}.issubset({f.name for f in msg.fields}), "XYZ fields missing"
        assert msg.width * msg.height > 0, "cloud is empty; check LiDAR/world geometry"
        assert msg.point_step >= 12, "invalid point_step"
        assert msg.row_step >= msg.width * msg.point_step, "invalid row_step"
        assert len(msg.data) >= msg.row_step * msg.height, "cloud payload is truncated"
        fields = {field.name: field for field in msg.fields}
        for name in ("x", "y", "z"):
            field = fields[name]
            assert field.datatype == 7 and field.count == 1, f"{name} must be a scalar FLOAT32"
            assert 0 <= field.offset <= msg.point_step - 4, f"{name} field offset is invalid"
        # Inspect up to 128 points, respecting per-row padding and actual field offsets.
        point_count = msg.width * msg.height
        sample_count = min(point_count, 128)
        format_code = ">f" if msg.is_bigendian else "<f"
        for sample in range(sample_count):
            index = sample * (point_count - 1) // max(1, sample_count - 1)
            row, column = divmod(index, msg.width)
            offset = row * msg.row_step + column * msg.point_step
            point = [struct.unpack_from(format_code, msg.data, offset + fields[name].offset)[0]
                     for name in ("x", "y", "z")]
            assert finite(point), "sampled XYZ contains NaN/Inf"
            distance = math.sqrt(sum(value*value for value in point))
            assert self.args.lidar_min_range - 1e-4 <= distance <= self.args.lidar_max_range + 1e-4, \
                f"sampled range {distance:.3f}m is outside the configured probe range"
        return stamp_seconds(msg.header.stamp)

    def image(self, msg, kind, frame):
        assert msg.header.frame_id == frame, f"expected frame {frame}"
        assert msg.width > 0 and msg.height > 0, "empty image"
        expected_encodings = {"rgb8", "bgr8", "rgba8", "bgra8"} if kind == "color" else {"32FC1", "16UC1"}
        assert msg.encoding in expected_encodings, f"unexpected image encoding {msg.encoding}"
        assert msg.step > 0 and len(msg.data) == msg.step * msg.height, "invalid image payload length"
        self.image_sizes[frame] = (msg.width, msg.height)
        if kind == "depth":
            bytes_per_pixel = 4 if msg.encoding == "32FC1" else 2
            assert msg.step >= msg.width * bytes_per_pixel, "depth row step is too small"
            format_code = (">" if msg.is_bigendian else "<") + ("f" if bytes_per_pixel == 4 else "H")
            valid_depth = False
            for row in range(msg.height):
                pixel_bytes = memoryview(msg.data)[row*msg.step:row*msg.step + msg.width*bytes_per_pixel]
                if any(math.isfinite(value) and value > 0 for (value,) in struct.iter_unpack(format_code, pixel_bytes)):
                    valid_depth = True
                    break
            assert valid_depth, "depth image contains no finite positive depth (all NaN/zero?)"
        return stamp_seconds(msg.header.stamp)

    def camera_info(self, msg, frame):
        assert msg.header.frame_id == frame, f"expected frame {frame}"
        assert msg.width > 0 and msg.height > 0, "empty calibration image size"
        assert len(msg.k) == 9 and msg.k[0] > 0 and msg.k[4] > 0 and abs(msg.k[8] - 1.0) < 1e-6, \
            "invalid camera intrinsics"
        assert finite(msg.k) and finite(msg.p), "calibration contains NaN/Inf"
        assert len(msg.p) == 12 and abs(msg.p[10] - 1.0) < 1e-6, "invalid projection matrix"
        assert 0 <= msg.k[2] < msg.width and 0 <= msg.k[5] < msg.height, \
            "principal point is outside the image"
        assert all(abs(msg.k[k] - msg.p[p]) < 1e-5 for k, p in ((0, 0), (4, 5), (2, 2), (5, 6))), \
            "K/P intrinsics differ for the ideal unrectified pinhole model"
        assert msg.p[0] > 0 and msg.p[5] > 0, "projection focal lengths must be positive"
        self.info_sizes[frame] = (msg.width, msg.height)
        return stamp_seconds(msg.header.stamp)

    def report(self):
        topics = {}
        for frame, dimensions in self.image_sizes.items():
            if frame in self.info_sizes and dimensions != self.info_sizes[frame]:
                self.errors.add(f"{frame}: image resolution differs from CameraInfo")
        for topic, record in self.records.items():
            count = record["count"]
            minimum = 1 if record["latched"] else 2
            if count < minimum:
                self.errors.add(f"{topic}: received {count} messages; require {minimum}")
            if not record["latched"] and record["stamp_advances"] < 1:
                self.errors.add(f"{topic}: no advancing simulation timestamps observed")
            span = (record["last_wall"] or 0) - (record["first_wall"] or 0)
            simulation_span = (record["last_stamp"] or 0) - (record["first_stamp"] or 0)
            topics[topic] = {
                "messages": count,
                "wall_hz": round((count-1) / span, 2) if count > 1 and span > 0 else None,
                "simulation_hz": round((count-1) / simulation_span, 2)
                if count > 1 and simulation_span > 0 else None,
                "last_simulation_stamp_s": record["last_stamp"],
            }
        for frame in sorted(self.frames):
            try:
                self.tf_buffer.lookup_transform("sim_world", frame, Time())
            except TransformException as exc:
                self.errors.add(f"TF sim_world -> {frame}: {exc}")
        return {"passed": not self.errors, "observation_wall_seconds": self.args.duration,
                "topics": topics, "errors": sorted(self.errors)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0, help="Wall-clock observation seconds")
    parser.add_argument("--skip-cameras", action="store_true")
    parser.add_argument("--skip-lidar", action="store_true")
    parser.add_argument("--lidar-min-range", type=float, default=0.1)
    parser.add_argument("--lidar-max-range", type=float, default=40.0)
    parser.add_argument("--output", type=Path, help="Optional JSON result path")
    args = parser.parse_args()
    if args.duration < 2:
        parser.error("--duration must be at least 2 seconds")
    if not 0 <= args.lidar_min_range < args.lidar_max_range:
        parser.error("LiDAR probe range must satisfy 0 <= min < max")
    os.environ.setdefault("ROS_DOMAIN_ID", "73")
    rclpy.init()
    node = StreamProbe(args)
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        report = node.report()
        result = json.dumps(report, indent=2)
        print(result)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(result + "\n", encoding="utf-8")
        return 0 if report["passed"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
