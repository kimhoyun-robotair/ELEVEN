"""Shared topic contracts and bounded, payload-aware observations."""

from collections import deque
from dataclasses import dataclass, field
import math
import struct
import time

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_msgs.msg import TFMessage


REQUIRED_EDGES = {
    ("odom", "base_footprint"),
    ("base_footprint", "base_link"),
    ("base_link", "front_camera_link"),
    ("front_camera_link", "front_camera_optical_frame"),
    ("base_link", "rear_camera_link"),
    ("rear_camera_link", "rear_camera_optical_frame"),
    ("base_link", "front_right_lidar_link"),
    ("base_link", "rear_left_lidar_link"),
}

TOPICS = {
    "/clock": (Clock, None),
    "/odom": (Odometry, "odom"),
    "/joint_states": (JointState, None),
    "/tf": (TFMessage, None),
    "/tf_static": (TFMessage, None),
}
for side in ("front", "rear"):
    for kind in ("color", "depth"):
        TOPICS[f"/{side}_camera/{kind}/image_raw"] = (Image, f"{side}_camera_optical_frame")
        TOPICS[f"/{side}_camera/{kind}/camera_info"] = (CameraInfo, f"{side}_camera_optical_frame")
for side in ("front_right", "rear_left"):
    TOPICS[f"/{side}_lidar/scan"] = (LaserScan, f"{side}_lidar_link")


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def finite(values):
    return all(math.isfinite(float(x)) for x in values)


def quat_valid(q):
    return finite((q.x, q.y, q.z, q.w)) and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 0.02


@dataclass
class Observation:
    count: int = 0
    last_wall: float = 0.0
    times: deque = field(default_factory=lambda: deque(maxlen=256))
    last_stamp: float | None = None
    stamp_advanced: bool = False
    last_advance_wall: float = 0.0
    errors: list = field(default_factory=list)
    details: dict = field(default_factory=dict)


class StreamCollector(Node):
    def __init__(self, name="amr_stream_monitor"):
        super().__init__(name)
        self.observations = {topic: Observation() for topic in TOPICS}
        self.tf_edges = set()
        self.tf_parents = {}
        self.latest_odom = None
        self.clock_seconds = None
        self.subscriptions_owned = []
        for topic, (message_type, frame) in TOPICS.items():
            qos = qos_profile_sensor_data
            if topic == "/tf_static":
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions_owned.append(self.create_subscription(
                message_type, topic, lambda msg, t=topic, f=frame: self._receive(t, f, msg), qos))

    def _receive(self, topic, expected_frame, msg):
        now = time.monotonic()
        state = self.observations[topic]
        state.count += 1
        state.last_wall = now
        state.times.append(now)
        errors = []
        details = {}
        stamp = None
        if hasattr(msg, "header"):
            stamp = stamp_seconds(msg.header.stamp)
            details["frame"] = msg.header.frame_id
            if expected_frame and msg.header.frame_id != expected_frame:
                errors.append(f"frame {msg.header.frame_id!r}, expected {expected_frame!r}")
        if isinstance(msg, Clock):
            stamp = stamp_seconds(msg.clock)
            self.clock_seconds = stamp
            details["sim_seconds"] = round(stamp, 4)
        elif isinstance(msg, Image):
            pixel_size = {"rgb8": 3, "rgba8": 4, "bgr8": 3, "bgra8": 4,
                          "32FC1": 4, "16UC1": 2}.get(msg.encoding)
            details.update(width=msg.width, height=msg.height, encoding=msg.encoding)
            if msg.width <= 0 or msg.height <= 0 or msg.step <= 0:
                errors.append("empty image dimensions")
            if len(msg.data) != msg.height * msg.step:
                errors.append("image data length != height * step")
            if pixel_size is None or msg.step < msg.width * pixel_size:
                errors.append("unsupported encoding or invalid row stride")
            if "/depth/" in topic:
                if msg.encoding not in ("32FC1", "16UC1"):
                    errors.append("depth encoding must be 32FC1 metres or 16UC1 millimetres")
                elif not errors:
                    # Sample at most 64 pixels; avoid copying megapixel messages.
                    prefix = ">" if msg.is_bigendian else "<"
                    code, size, scale = ("f", 4, 1.0) if msg.encoding == "32FC1" else ("H", 2, 0.001)
                    data = memoryview(msg.data)
                    depths = []
                    for iy in range(8):
                        y = min(msg.height-1, (iy * msg.height + msg.height//2)//8)
                        for ix in range(8):
                            x = min(msg.width-1, (ix * msg.width + msg.width//2)//8)
                            value = struct.unpack_from(prefix + code, data, y * msg.step + x * size)[0] * scale
                            if math.isfinite(value) and value > 0:
                                depths.append(value)
                    details["finite_depth_samples"] = len(depths)
                    if depths:
                        details["sample_depth_m"] = [round(min(depths), 3), round(max(depths), 3)]
            elif msg.encoding not in ("rgb8", "rgba8", "bgr8", "bgra8"):
                errors.append("color encoding is not RGB/BGR")
        elif isinstance(msg, CameraInfo):
            details.update(width=msg.width, height=msg.height)
            if msg.width <= 0 or msg.height <= 0 or not finite(msg.k) or msg.k[0] <= 0 or msg.k[4] <= 0:
                errors.append("invalid pinhole calibration")
        elif isinstance(msg, LaserScan):
            ranges = list(msg.ranges)
            details.update(beams=len(ranges), finite_returns=sum(math.isfinite(v) for v in ranges))
            if not ranges or not finite((msg.angle_min, msg.angle_max, msg.angle_increment,
                                         msg.range_min, msg.range_max)):
                errors.append("empty scan or nonfinite scan metadata")
            elif msg.angle_increment <= 0 or msg.range_min < 0 or msg.range_max <= msg.range_min:
                errors.append("invalid scan angle/range limits")
            elif abs(msg.angle_min + (len(ranges)-1)*msg.angle_increment - msg.angle_max) > msg.angle_increment*1.1:
                errors.append("scan beam count inconsistent with angular limits")
            if any(math.isfinite(v) and not (msg.range_min-1e-4 <= v <= msg.range_max+1e-4) for v in ranges):
                errors.append("finite scan return outside declared limits")
        elif isinstance(msg, Odometry):
            self.latest_odom = msg
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            details.update(child_frame=msg.child_frame_id, xy=[round(p.x, 3), round(p.y, 3)],
                           vx=round(msg.twist.twist.linear.x, 3), wz=round(msg.twist.twist.angular.z, 3))
            if msg.child_frame_id != "base_footprint":
                errors.append("odom child_frame_id must be base_footprint")
            if not finite((p.x, p.y, p.z, msg.twist.twist.linear.x, msg.twist.twist.angular.z)) or not quat_valid(q):
                errors.append("invalid odometry pose or twist")
        elif isinstance(msg, JointState):
            details["joint_names"] = list(msg.name)
            if not {"left_drive_joint", "right_drive_joint"}.issubset(msg.name):
                errors.append("drive joints absent")
            if len(msg.position) != len(msg.name) or not finite(msg.position):
                errors.append("joint positions missing or invalid")
            for name in ("velocity", "effort"):
                values = getattr(msg, name)
                if values and (len(values) != len(msg.name) or not finite(values)):
                    errors.append(f"invalid joint {name}")
        elif isinstance(msg, TFMessage):
            details["transforms"] = len(msg.transforms)
            if not msg.transforms:
                errors.append("empty TFMessage")
            for transform in msg.transforms:
                parent, child = transform.header.frame_id, transform.child_frame_id
                if not parent or not child or parent == child:
                    errors.append("invalid TF parent/child")
                if child in self.tf_parents and self.tf_parents[child] != parent:
                    errors.append(f"multiple TF parents for {child}")
                self.tf_parents[child] = parent
                self.tf_edges.add((parent, child))
                p, q = transform.transform.translation, transform.transform.rotation
                if not finite((p.x, p.y, p.z)) or not quat_valid(q):
                    errors.append(f"invalid TF pose for {child}")
            if topic == "/tf" and msg.transforms:
                stamp = max(stamp_seconds(t.header.stamp) for t in msg.transforms)
        if stamp is not None:
            if state.last_stamp is not None:
                state.stamp_advanced |= stamp > state.last_stamp
                if stamp > state.last_stamp:
                    state.last_advance_wall = now
                if stamp < state.last_stamp - 1e-6:
                    errors.append("timestamp moved backwards; restart monitor after resetting simulation")
            state.last_stamp = stamp
        state.errors = errors
        state.details = details

    def snapshot(self, stale_after=5.0):
        now = time.monotonic()
        result = {}
        for topic, state in self.observations.items():
            recent = [t for t in state.times if now-t <= 10.0]
            hz = (len(recent)-1)/(recent[-1]-recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
            age = now - state.last_wall if state.count else None
            errors = list(state.errors)
            if not state.count:
                errors.append("no messages")
            elif topic != "/tf_static" and age > stale_after:
                errors.append("stream stale")
            elif topic != "/tf_static" and state.stamp_advanced and now-state.last_advance_wall > stale_after:
                errors.append("message timestamps stopped advancing")
            result[topic] = {"count": state.count, "wall_hz": round(hz, 2),
                             "age_wall_s": round(age, 2) if age is not None else None,
                             "stamp_advanced": state.stamp_advanced,
                             "errors": errors, "details": state.details}
        return result

    def problems(self, require_advance=True):
        result = []
        for topic, state in self.snapshot().items():
            result.extend(f"{topic}: {error}" for error in state["errors"])
            if require_advance and topic != "/tf_static" and not state["stamp_advanced"]:
                result.append(f"{topic}: header/clock timestamps have not advanced")
        for parent, child in sorted(REQUIRED_EDGES - self.tf_edges):
            result.append(f"TF missing: {parent} -> {child}")
        return result
