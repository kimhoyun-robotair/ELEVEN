#!/usr/bin/env python3
"""Isaac Sim 5.1.0 standalone AMR world and ROS 2 Jazzy data bridge.

Run with scripts/run_sim.sh (Isaac's Python 3.11), never system Python.
All sensor values come from RTX rendering or the live PhysX collision scene.
The packaged USD remains unchanged: runtime changes use its session layer.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from sim.control import DriveLimiter


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--headless", action="store_true", help="Still renders camera products on the GPU")
    p.add_argument("--duration", type=float, default=0, help="Simulation seconds, 0 runs until closed")
    p.add_argument("--payload-kg", type=float, default=None,
                   help="Centered secured payload, 0..250 kg; omitted preserves the selected world payload")
    p.add_argument("--config", type=Path, default=PROJECT / "config/robot.json")
    p.add_argument("--world", type=Path, default=PROJECT / "assets/test_world.usd")
    p.add_argument("--report", type=Path, default=PROJECT / "logs/runtime_report.json")
    p.add_argument("--startup-timeout", type=float, default=15, help="Sensor warm-up limit in simulation seconds")
    p.add_argument("--no-realtime", action="store_true", help="Run as fast as hardware allows")
    p.add_argument("--caster-yaw-deg", type=float, default=None, help="Initial swivel angle for repeatable caster startup checks")
    p.add_argument("--physics-trace", action="store_true", help="Include half-second wheel pose samples in the report")
    a = p.parse_args()
    if not a.config.is_file() or not a.world.is_file():
        p.error("Config or world USD missing. Re-extract the complete package or build the assets.")
    cfg = json.loads(a.config.read_text())
    if a.payload_kg is not None and (not math.isfinite(a.payload_kg) or
                                    not 0 <= a.payload_kg <= cfg["max_payload_kg"]):
        p.error("--payload-kg must lie within the configured 0..250 kg range")
    if not math.isfinite(a.duration) or a.duration < 0 or a.startup_timeout <= 0:
        p.error("duration must be nonnegative and startup-timeout positive")
    if a.caster_yaw_deg is not None and not math.isfinite(a.caster_yaw_deg):
        p.error("caster-yaw-deg must be finite")
    return a, cfg


def run(a, cfg, report):
    # SimulationApp MUST precede all omni/pxr/Isaac extension imports.
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": a.headless, "renderer": "RayTracedLighting",
                         "width": 1280, "height": 720, "anti_aliasing": 1,
                         "fast_shutdown": True})
    node = None
    sim = None
    cameras = []
    try:
        import numpy as np
        import carb
        import omni.usd
        import omni.physx
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.sensors.camera")
        app.update()
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from builtin_interfaces.msg import Time
            from geometry_msgs.msg import Twist, TransformStamped
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import Image, CameraInfo, JointState, LaserScan
            from tf2_msgs.msg import TFMessage
        except ImportError as exc:
            raise RuntimeError("Bundled Python 3.11 ROS imports failed. Start from a fresh terminal "
                               "using scripts/run_sim.sh; do not source /opt/ros/jazzy here.") from exc
        from isaacsim.core.api import SimulationContext
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.stage import open_stage, is_stage_loading
        from isaacsim.sensors.camera import Camera
        from model.build_assets import apply_payload
        from sim.camera_frames import camera_sample
        from sim.physics_health import PhysicsHealth, cylinder_bottom

        if not open_stage(str(a.world.resolve())):
            raise RuntimeError(f"Could not open {a.world}")
        loading_start = time.monotonic()
        while is_stage_loading():
            app.update()
            if time.monotonic() - loading_start > 120:
                raise RuntimeError("USD loading timed out after 120 seconds")
        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        root = "/World/Robot"
        base_path = f"{root}/base_link"
        required = [root, base_path, f"{root}/left_drive", f"{root}/right_drive"]
        required += [f"{base_path}/{s}_camera_link/camera" for s in ("front", "rear")]
        required += [f"{base_path}/{s}_lidar_link" for s in ("front_right", "rear_left")]
        absent = [p for p in required if not stage.GetPrimAtPath(p).IsValid()]
        if absent:
            raise RuntimeError(f"World has missing required robot prims: {absent}")
        if not stage.GetPrimAtPath(base_path).HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("base_link must be a dynamic rigid body")
        if a.payload_kg is None:
            authored_mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(base_path)).GetMassAttr().Get()
            a.payload_kg = max(0.0, float(authored_mass) - float(cfg["base_mass_kg"]))
        apply_payload(stage, a.payload_kg, cfg, robot_path=root)

        phz, rhz = int(cfg["simulation"]["physics_hz"]), int(cfg["simulation"]["render_hz"])
        camera_cfg, lidar_cfg = cfg["camera"], cfg["lidar"]
        if phz <= 0 or rhz <= 0 or phz % rhz or rhz % int(camera_cfg["hz"]):
            raise ValueError("physics_hz must be a multiple of render_hz; render_hz a multiple of camera.hz")
        if phz % int(lidar_cfg["hz"]):
            raise ValueError("physics_hz must be a multiple of lidar.hz")
        dt = 1.0 / phz
        sim = SimulationContext(physics_dt=dt, rendering_dt=1.0 / rhz,
                                stage_units_in_meters=1.0, backend="numpy", device="cpu",
                                physics_prim_path="/World/PhysicsScene")
        physics = sim.get_physics_context()
        # CPU PhysX supports scene queries. GPU is still used for camera rendering.
        physics.enable_gpu_dynamics(False)
        physics.enable_fabric(False)
        carb.settings.get_settings().set_bool("/physics/updateToUsd", True)
        carb.settings.get_settings().set_bool("/physics/updateVelocitiesToUsd", True)
        sim.set_block_on_render(True)
        robot = SingleArticulation(prim_path=root, name="photo_amr", reset_xform_properties=False)
        sim.reset()
        robot.initialize()
        if a.caster_yaw_deg is not None:
            caster_indices = np.array([robot.get_dof_index(f"caster_{corner}_swivel_joint") for corner in ("fl", "fr", "rl", "rr")])
            robot.set_joint_positions(np.full(4, math.radians(a.caster_yaw_deg)), joint_indices=caster_indices)
        wheel_names = ["left_drive", "right_drive"] + [f"caster_{corner}_wheel" for corner in ("fl", "fr", "rl", "rr")]
        wheel_view = RigidPrim([f"{root}/{name}" for name in wheel_names], name="wheel_poses",
                               reset_xform_properties=False, prepare_contact_sensors=False)
        wheel_view.initialize()
        drive_names = ["left_drive_joint", "right_drive_joint"]
        indices = np.array([robot.get_dof_index(n) for n in drive_names], dtype=np.int32)
        all_joint_names = list(robot.dof_names)
        if len(all_joint_names) != 10:
            raise RuntimeError(f"Expected 10 DOFs (2 powered + 8 passive), received {all_joint_names}")

        d = cfg["drive"]
        limiter = DriveLimiter(radius=d["radius_m"], track=d["track_m"],
            max_speed=d["max_speed_mps"], max_accel=d["max_accel_mps2"],
            max_yaw_rate=d["max_yaw_rate_rps"], max_yaw_accel=d["max_yaw_accel_rps2"],
            timeout=d["command_timeout_s"])
        rclpy.init(args=[])
        node = rclpy.create_node("photo_amr_isaac51")
        data_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pubs = {
            "clock": node.create_publisher(Clock, "/clock", reliable_qos),
            "odom": node.create_publisher(Odometry, "/odom", reliable_qos),
            "joint_states": node.create_publisher(JointState, "/joint_states", reliable_qos),
            "tf": node.create_publisher(TFMessage, "/tf", reliable_qos),
            "tf_static": node.create_publisher(TFMessage, "/tf_static", static_qos),
        }
        current_time = [float(sim.current_time)]
        ready = [False]

        def command(msg):
            # Reject commands during warm-up, so a stale teleop cannot launch the robot.
            if not ready[0]:
                return
            limiter.command(float(msg.linear.x), float(msg.angular.z),
                            current_time[0], time.monotonic())

        subscription = node.create_subscription(Twist, "/cmd_vel", command, reliable_qos)

        def stamp(seconds):
            ns = max(0, round(float(seconds) * 1e9))
            return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

        def world_matrix(path, cache=None):
            c = cache if cache is not None else UsdGeom.XformCache(Usd.TimeCode.Default())
            return c.GetLocalToWorldTransform(stage.GetPrimAtPath(path))

        def transform(parent, child, matrix, seconds):
            m = TransformStamped()
            m.header.stamp, m.header.frame_id, m.child_frame_id = stamp(seconds), parent, child
            t = matrix.ExtractTranslation()
            q = matrix.ExtractRotationQuat().GetNormalized()
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t)
            m.transform.rotation.w = float(q.GetReal())
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z = map(float, q.GetImaginary())
            return m

        def relative(child_path, parent_path, cache):
            # Gf uses row vectors: world(child) * inverse(world(parent)).
            return world_matrix(child_path, cache) * world_matrix(parent_path, cache).GetInverse()

        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        static_transforms = []
        report["camera_frames"] = {"front": 0, "rear": 0}
        report["camera_valid_depth_frames"] = {"front": 0, "rear": 0}
        for side in ("front", "rear"):
            link = f"{side}_camera_link"
            path = f"{base_path}/{link}"
            static_transforms.append(transform("base_link", link, relative(path, base_path, cache), 0))
            # USD camera: -Z forward, +Y up. ROS optical: +Z forward, +Y down.
            usd_to_optical = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
            optical_in_link = usd_to_optical * relative(f"{path}/camera", path, cache)
            static_transforms.append(transform(link, f"{side}_camera_optical_frame", optical_in_link, 0))
            cam = Camera(prim_path=f"{path}/camera", name=f"{side}_rgbd",
                         frequency=int(camera_cfg["hz"]),
                         resolution=(int(camera_cfg["width"]), int(camera_cfg["height"])))
            cam.initialize()
            cam.add_distance_to_image_plane_to_frame()
            # Pinhole intrinsics already authored in USD; publish their actual values.
            cam.set_clipping_range(0.1, 20.0)
            cameras.append({"name": side, "camera": cam, "last_frame": None,
                            "last_publish_time": -math.inf, "count": 0,
                            "valid_depth_frames": 0, "textured_rgb_frames": 0})
            for channel in ("color", "depth"):
                prefix = f"/{side}_camera/{channel}"
                pubs[prefix + "/image_raw"] = node.create_publisher(Image, prefix + "/image_raw", data_qos)
                pubs[prefix + "/camera_info"] = node.create_publisher(CameraInfo, prefix + "/camera_info", data_qos)
        for side in ("front_right", "rear_left"):
            link = f"{side}_lidar_link"
            static_transforms.append(transform("base_link", link, relative(f"{base_path}/{link}", base_path, cache), 0))
            topic = f"/{side}_lidar/scan"
            pubs[topic] = node.create_publisher(LaserScan, topic, data_qos)
        pubs["tf_static"].publish(TFMessage(transforms=static_transforms))

        # Publish joint-link transforms from actual physical poses, including free casters.
        moving_links = [("base_link", "left_drive"), ("base_link", "right_drive")]
        for corner in ("fl", "fr", "rl", "rr"):
            moving_links += [("base_link", f"caster_{corner}_swivel"),
                             (f"caster_{corner}_swivel", f"caster_{corner}_wheel")]
        previous_pose: list[tuple | None] = [None]
        z_values = []
        report["physics_samples"] = []
        last_physics_sample = [-math.inf]
        last_trace_sample = [-math.inf]
        physics_health = PhysicsHealth()

        def publish_state(seconds):
            c = UsdGeom.XformCache(Usd.TimeCode.Default())
            bm = world_matrix(base_path, c)
            pos = np.array(bm.ExtractTranslation(), dtype=float)
            # Column 0 of the conventional matrix is row 0 of Gf's matrix.
            yaw = math.atan2(float(bm[0][1]), float(bm[0][0]))
            if not np.isfinite(pos).all() or not math.isfinite(yaw):
                raise RuntimeError("Physics produced a non-finite base pose")
            if seconds > 2 and (pos[2] < 0.06 or pos[2] > 1.0):
                raise RuntimeError(f"Robot height out of bounds: base_link z={pos[2]:.4f} m")
            z_values.append(float(pos[2]))
            if len(z_values) > 1200:
                del z_values[:600]
            odom_matrix = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)))
            odom_matrix.SetTranslateOnly(Gf.Vec3d(float(pos[0]), float(pos[1]), 0))
            base_relative = bm * odom_matrix.GetInverse()
            tf = [transform("odom", "base_footprint", odom_matrix, seconds),
                  transform("base_footprint", "base_link", base_relative, seconds)]
            for parent, child in moving_links:
                pp = base_path if parent == "base_link" else f"{root}/{parent}"
                tf.append(transform(parent, child, relative(f"{root}/{child}", pp, c), seconds))
            pubs["tf"].publish(TFMessage(transforms=tf))
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp(seconds), "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x, odom.pose.pose.position.y = float(pos[0]), float(pos[1])
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            old = previous_pose[0]
            if old and seconds > old[0]:
                delta_t = seconds - old[0]
                vel = (pos[:2] - old[1][:2]) / delta_t
                odom.twist.twist.linear.x = float(math.cos(yaw) * vel[0] + math.sin(yaw) * vel[1])
                odom.twist.twist.linear.y = float(-math.sin(yaw) * vel[0] + math.cos(yaw) * vel[1])
                odom.twist.twist.angular.z = math.atan2(math.sin(yaw - old[2]), math.cos(yaw - old[2])) / delta_t
            previous_pose[0] = (seconds, pos.copy(), yaw)
            # Zero covariance intentionally identifies ideal ground-truth odometry.
            pubs["odom"].publish(odom)
            joints = JointState()
            joints.header.stamp = stamp(seconds)
            joints.name = all_joint_names
            joints.position = np.asarray(robot.get_joint_positions(), dtype=float).tolist()
            joints.velocity = np.asarray(robot.get_joint_velocities(), dtype=float).tolist()
            # Effort left empty: a target is not measured actuator torque.
            pubs["joint_states"].publish(joints)
            report["end_base_position_m"] = pos.tolist()
            report["end_yaw_rad"] = yaw
            if seconds - last_physics_sample[0] >= 0.1 - dt / 2:
                positions, orientations = wheel_view.get_world_poses()
                wheel_states = {}
                for index, name in enumerate(wheel_names):
                    wc = cfg["drive"] if name.endswith("drive") else cfg["caster"]
                    usd_pos = world_matrix(f"{root}/{name}", c).ExtractTranslation()
                    wheel_states[name] = {"bottom_z_m": cylinder_bottom(positions[index], orientations[index], wc["radius_m"], wc["width_m"]),
                                          "position_m": positions[index].tolist(),
                                          "usd_physics_position_error_m": float(np.linalg.norm(np.asarray(usd_pos) - positions[index]))}
                up = bm.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
                sample = {"sim_time": seconds, "base_position_m": pos.tolist(),
                          "tilt_deg": math.degrees(math.acos(max(-1, min(1, up[2])))), "wheels": wheel_states}
                report["physics_latest"] = sample
                if a.physics_trace and seconds - last_trace_sample[0] >= 0.5 - dt / 2:
                    report["physics_samples"].append(sample)
                    last_trace_sample[0] = seconds
                last_physics_sample[0] = seconds
                try:
                    physics_health.update(seconds - start_sim, sample["tilt_deg"],
                                          {name: state["bottom_z_m"] for name, state in wheel_states.items()})
                finally:
                    report["physics_checks"] = {"stable": physics_health.ready, "max_tilt_deg": physics_health.max_tilt_deg,
                                                "min_wheel_bottom_m": dict(physics_health.min_bottom_m)}
            if "start_base_position_m" not in report:
                report["start_base_position_m"] = pos.tolist()

        def camera_info(camera, seconds, side):
            k = np.asarray(camera.get_intrinsics_matrix(), dtype=np.float64)
            info = CameraInfo()
            info.header.stamp = stamp(seconds)
            info.header.frame_id = f"{side}_camera_optical_frame"
            info.width, info.height = int(camera_cfg["width"]), int(camera_cfg["height"])
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = k.reshape(-1).tolist()
            info.r = np.eye(3).reshape(-1).tolist()
            info.p = np.column_stack((k, np.zeros(3))).reshape(-1).tolist()
            return info

        def publish_cameras(seconds):
            width, height = int(camera_cfg["width"]), int(camera_cfg["height"])
            for entry in cameras:
                cam, side = entry["camera"], entry["name"]
                frame = cam.get_current_frame()
                try:
                    sample = camera_sample(frame, width, height, seconds, dt)
                except ValueError as exc:
                    raise RuntimeError(f"{side} camera: {exc}") from exc
                if sample is None:
                    continue
                key, timestamp, rgba, depth = sample
                if key == entry["last_frame"]:
                    continue
                if timestamp <= entry["last_publish_time"]:
                    continue
                entry["last_frame"] = key
                entry["last_publish_time"] = timestamp
                rgb = np.ascontiguousarray(rgba[:, :, :3], dtype=np.uint8)
                depth = np.ascontiguousarray(depth, dtype="<f4")
                valid = np.isfinite(depth) & (depth >= camera_cfg["min_depth_m"]) & (depth <= camera_cfg["max_depth_m"])
                depth = depth.copy()
                depth[~valid] = np.nan
                info = camera_info(cam, timestamp, side)
                for channel, data, encoding, bpp in (("color", rgb, "rgb8", 3), ("depth", depth, "32FC1", 4)):
                    msg = Image()
                    msg.header = info.header
                    msg.width, msg.height = width, height
                    msg.encoding, msg.is_bigendian, msg.step = encoding, 0, width * bpp
                    msg.data = data.tobytes()
                    topic = f"/{side}_camera/{channel}"
                    pubs[topic + "/image_raw"].publish(msg)
                    pubs[topic + "/camera_info"].publish(info)
                entry["count"] += 1
                entry["valid_depth_frames"] += int(np.any(valid))
                entry["textured_rgb_frames"] += int(float(np.std(rgb.astype(np.float32))) > 1.0)
                report["camera_frames"][side] = entry["count"]
                report["camera_valid_depth_frames"][side] = entry["valid_depth_frames"]

        query = omni.physx.get_physx_scene_query_interface()
        scan_counts = {s: 0 for s in ("front_right", "rear_left")}
        finite_scan_counts = {s: 0 for s in scan_counts}
        report["scan_frames"], report["scan_finite_frames"] = scan_counts, finite_scan_counts
        samples = int(lidar_cfg["samples"])
        if samples < 2:
            raise ValueError("lidar.samples must be at least 2")
        half_fov = math.radians(float(lidar_cfg["fov_deg"])) / 2
        angles = np.linspace(-half_fov, half_fov, samples)
        local_dirs = [Gf.Vec3d(math.cos(v), math.sin(v), 0) for v in angles]

        def publish_scans(seconds):
            c = UsdGeom.XformCache(Usd.TimeCode.Default())
            for side in scan_counts:
                matrix = world_matrix(f"{base_path}/{side}_lidar_link", c)
                origin = carb.Float3(*map(float, matrix.ExtractTranslation()))
                ranges = []
                for direction in local_dirs:
                    world_dir = matrix.TransformDir(direction).GetNormalized()
                    nearest = [math.inf]
                    def hit_callback(hit):
                        # raycast_all is unsorted; inspect every hit before choosing a minimum.
                        # Robot shapes are removed just as in a software self-filter.
                        path = str(hit.collision)
                        if not (path == root or path.startswith(root + "/")):
                            distance = float(hit.distance)
                            if 0 <= distance < nearest[0]:
                                nearest[0] = distance
                        return True
                    query.raycast_all(origin, carb.Float3(*map(float, world_dir)),
                                      float(lidar_cfg["range_max_m"]), hit_callback)
                    distance = nearest[0]
                    if distance < lidar_cfg["range_min_m"]:
                        distance = math.nan  # Too close is invalid, never a fabricated minimum return.
                    ranges.append(distance)
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp(seconds), f"{side}_lidar_link"
                scan.angle_min, scan.angle_max = float(angles[0]), float(angles[-1])
                scan.angle_increment = float(angles[1] - angles[0])
                scan.scan_time, scan.time_increment = 1.0 / lidar_cfg["hz"], 0.0
                scan.range_min, scan.range_max = float(lidar_cfg["range_min_m"]), float(lidar_cfg["range_max_m"])
                scan.ranges = ranges
                # No intensity/reflection model: the optional intensities array remains empty.
                pubs[f"/{side}_lidar/scan"].publish(scan)
                scan_counts[side] += 1
                finite_scan_counts[side] += int(any(math.isfinite(v) for v in ranges))

        if not a.headless:
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=np.array([2.1, -2.4, 1.7]), target=np.array([0, 0, 0.15]))
        report.update({"phase": "running", "python_version": sys.version.split()[0],
                       "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
                       "payload_kg": a.payload_kg, "initial_caster_yaw_deg": a.caster_yaw_deg, "joint_names": all_joint_names,
                       "odom_source": "PhysX ground truth projected to base_footprint",
                       "lidar_source": "PhysX raycast_all, ideal instantaneous scan, robot self-filter",
                       "camera_source": "RTX RGBA and distance_to_image_plane annotators"})
        stop_requested = [False]
        signal.signal(signal.SIGINT, lambda *_: stop_requested.__setitem__(0, True))
        signal.signal(signal.SIGTERM, lambda *_: stop_requested.__setitem__(0, True))
        render_every, scan_every = phz // rhz, phz // int(lidar_cfg["hz"])
        state_every = max(1, phz // 60)
        frame_idx = 0
        start_sim, start_wall = float(sim.current_time), time.monotonic()
        last_status_wall = start_wall
        while app.is_running() and rclpy.ok() and not stop_requested[0]:
            if not sim.is_playing():
                if sim.is_stopped():
                    print("Timeline stopped. Relaunch run_sim.sh for a full reset.", flush=True)
                    break
                app.update()
                rclpy.spin_once(node, timeout_sec=0.0)
                # Clear stale commands across GUI pause/resume.
                limiter.requested_linear = limiter.requested_angular = 0.0
                limiter.last_sim = limiter.last_wall = -math.inf
                start_wall = time.monotonic() - (float(sim.current_time) - start_sim)
                continue
            current_time[0] = float(sim.current_time)
            rclpy.spin_once(node, timeout_sec=0.0)
            left, right = limiter.step(dt, current_time[0], time.monotonic())
            robot.apply_action(ArticulationAction(joint_velocities=np.array([left, right]), joint_indices=indices))
            sim.step(render=False)
            # Reading transforms from USD is intentional; force their physical state current.
            omni.physx.get_physx_interface().update_transformations(False, True, True)
            current_time[0] = float(sim.current_time)
            seconds = current_time[0]
            frame_idx += 1
            clock = Clock()
            clock.clock = stamp(seconds)
            pubs["clock"].publish(clock)
            if frame_idx % state_every == 0:
                publish_state(seconds)
            if frame_idx % scan_every == 0:
                publish_scans(seconds)
            if frame_idx % render_every == 0:
                sim.render()
                publish_cameras(seconds)
            elapsed = seconds - start_sim
            if not ready[0]:
                sensors_ready = all(e["valid_depth_frames"] > 0 and e["textured_rgb_frames"] > 0 for e in cameras)
                sensors_ready &= all(finite_scan_counts[s] > 0 for s in scan_counts)
                if elapsed >= 2.0 and sensors_ready and physics_health.ready:
                    ready[0] = True
                    report["startup_checks_passed"] = True
                    report["settled_base_z_m"] = report["end_base_position_m"][2]
                    print("AMR_READY: RGB, depth, scans, joints and odometry are live. /cmd_vel enabled.", flush=True)
                elif elapsed > a.startup_timeout:
                    raise RuntimeError(f"Startup check failed: sensors_ready={sensors_ready}, "
                                       f"physics_stable={physics_health.ready}, camera_frames={report['camera_frames']}")
            if time.monotonic() - last_status_wall > 10:
                print(f"AMR sim={elapsed:.1f}s ready={ready[0]} cmd_count={limiter.accepted} "
                      f"camera_frames={[e['count'] for e in cameras]} scans={scan_counts} "
                      f"tilt_deg={report['physics_latest']['tilt_deg']:.3f}", flush=True)
                last_status_wall = time.monotonic()
            if a.duration and elapsed >= a.duration:
                break
            if not a.no_realtime:
                # Short sleeps only; rendering-limited hardware naturally runs slower.
                delay = start_wall + elapsed - time.monotonic()
                if delay > 0:
                    time.sleep(min(delay, dt))
        report.update({"phase": "completed", "simulated_seconds": float(sim.current_time) - start_sim,
                       "startup_checks_passed": ready[0], "accepted_commands": limiter.accepted,
                       "rejected_commands": limiter.rejected, "watchdog_stops": limiter.watchdog_stops,
                       "camera_frames": {e["name"]: e["count"] for e in cameras},
                       "camera_valid_depth_frames": {e["name"]: e["valid_depth_frames"] for e in cameras},
                       "scan_frames": scan_counts, "scan_finite_frames": finite_scan_counts,
                       "recent_base_z_min_m": min(z_values) if z_values else None,
                       "recent_base_z_max_m": max(z_values) if z_values else None})
        if not ready[0]:
            raise RuntimeError("Simulation ended before startup checks passed")
        # This is a startup/data check only. External smoke_test.py verifies commanded motion.
        report["runtime_startup_passed"] = True
    except BaseException as exc:
        report.update({"phase": "failed", "error": f"{type(exc).__name__}: {exc}"})
        traceback.print_exc()
        raise
    finally:
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"Runtime report: {a.report}", flush=True)
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        if sim is not None:
            sim.stop()
        app.app.post_quit(1 if report.get("error") else 0)
        app.close()


def main():
    a, cfg = arguments()
    report = {"runtime_startup_passed": False, "startup_checks_passed": False,
              "phase": "initializing", "target": "Isaac Sim 5.1.0 / Ubuntu 24.04 / ROS 2 Jazzy",
              "test_scope": "live startup and stream checks; motion verified by separate ROS smoke test"}
    code = 0
    try:
        run(a, cfg, report)
    except Exception as exc:
        report.update({"phase": "failed", "error": f"{type(exc).__name__}: {exc}"})
        traceback.print_exc()
        code = 1
    finally:
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"Runtime report: {a.report}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
