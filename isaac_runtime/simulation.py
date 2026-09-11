#!/usr/bin/env python3
"""AMR multi-floor simulation and ROS 2 Jazzy data bridge for Isaac Sim 5.1.0.

Run with scripts/sim (Isaac's Python 3.11), never system Python.
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
from isaac_runtime.control import DriveLimiter


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--headless", action="store_true", help="Still renders camera products on the GPU")
    p.add_argument("--duration", type=float, default=0, help="Simulation seconds after sensor readiness; 0 runs until closed")
    p.add_argument("--config", type=Path, default=PROJECT / "config/robot.json")
    p.add_argument("--scene", choices=("office", "house"), default="office")
    p.add_argument("--robot", choices=("amr", "locomanipulator"), default="amr")
    p.add_argument("--spawn", nargs=4, type=float, metavar=("X", "Y", "Z", "YAW"), help="Initial base-footprint pose in meters and yaw degrees")
    p.add_argument("--inspection", action="store_true", help="Isolated AMR turntable in Isaac GUI")
    p.add_argument("--capture-orbit", type=Path, help="Write 360-degree PNG frames from the actual USD")
    p.add_argument("--report", type=Path, default=PROJECT / ".runtime/logs/runtime_report.json")
    p.add_argument("--startup-timeout", type=float, default=180, help="Sensor warm-up limit in wall seconds; includes cold RTX compilation")
    p.add_argument("--no-realtime", action="store_true", help="Run as fast as hardware allows")
    p.add_argument("--caster-yaw-deg", type=float, default=None, help="Initial swivel angle for repeatable caster startup checks")
    p.add_argument("--physics-trace", action="store_true", help="Include half-second wheel pose samples in the report")
    a, kit_args = p.parse_known_args()
    sys.argv = [sys.argv[0], *kit_args]
    a.world = PROJECT / "assets/scenes" / f"{a.scene}.usda"
    if not a.config.is_file() or not a.world.is_file():
        p.error("Config or world USD missing. Re-extract the complete package or build the assets.")
    cfg = json.loads(a.config.read_text())
    if not math.isfinite(a.duration) or a.duration < 0 or a.startup_timeout <= 0:
        p.error("duration must be nonnegative and startup-timeout positive")
    if a.caster_yaw_deg is not None and not math.isfinite(a.caster_yaw_deg):
        p.error("caster-yaw-deg must be finite")
    if a.spawn and not all(math.isfinite(v) for v in a.spawn):
        p.error("spawn values must be finite")
    return a, cfg


def run(a, cfg, report):
    # SimulationApp MUST precede all omni/pxr/Isaac extension imports.
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": a.headless, "renderer": "RayTracedLighting",
                         "width": 1280, "height": 720, "anti_aliasing": 1,
                         "fast_shutdown": True, "extra_args": [
                             f"--portable-root={PROJECT / '.runtime/kit'}",
                             "--/app/settings/persistent=false",
                             f"--/log/file={PROJECT / '.runtime/logs/kit.log'}"]})
    node = None
    sim = None
    cameras = []
    sensors = elevators = views = arm = None
    try:
        import numpy as np
        import carb
        import omni.usd
        import omni.physx
        from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdLux, PhysicsSchemaTools
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.sensors.camera")
        enable_extension("isaacsim.sensors.physics")
        enable_extension("isaacsim.sensors.rtx")
        app.update()
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from builtin_interfaces.msg import Time
            from geometry_msgs.msg import Twist, TransformStamped
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import Image, CameraInfo, JointState, LaserScan, PointCloud2
            from tf2_msgs.msg import TFMessage
            from std_msgs.msg import String
        except ImportError as exc:
            raise RuntimeError("Bundled Python 3.11 ROS imports failed. Start from a fresh terminal "
                               "using scripts/sim; do not source /opt/ros/jazzy here.") from exc
        from isaacsim.core.api import SimulationContext
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.stage import open_stage, is_stage_loading
        from isaacsim.sensors.camera import Camera
        from isaac_runtime.camera_frames import camera_sample
        from isaac_runtime.physics_health import PhysicsHealth, cylinder_bottom

        if not open_stage(str(a.world.resolve())):
            raise RuntimeError(f"Could not open {a.world}")
        loading_start = time.monotonic()
        while is_stage_loading():
            app.update()
            if time.monotonic() - loading_start > 120:
                raise RuntimeError("USD loading timed out after 120 seconds")
        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        import omni.timeline
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_end_time(86400.0)
        timeline.set_looping(False)
        # A flush landing sill avoids trapping the AMR's 40 mm passive casters.
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh) and 'LandingThreshold' in prim.GetName():
                UsdGeom.Xformable(prim).AddTranslateOp(opSuffix='amrFlushSill').Set(Gf.Vec3d(0, 0, -.007))
        route = json.loads((PROJECT / 'src/aprl_robot_sim/examples' / f'{a.scene}.json').read_text())
        if a.spawn:
            route['spawn'] = a.spawn
        if a.inspection:
            for path in ('/World/Building', '/World/Furniture', '/World/Elevators', '/World/Lighting'):
                stage.GetPrimAtPath(path).SetActive(False)
            PhysicsSchemaTools.addGroundPlane(stage, '/World/InspectionFloor', 'Z', 20.0,
                                              Gf.Vec3f(0, 0, 0), Gf.Vec3f(.16, .18, .21))
            key = UsdLux.RectLight.Define(stage, '/World/InspectionKey')
            key.CreateWidthAttr(2.0)
            key.CreateHeightAttr(2.0)
            key.CreateIntensityAttr(3000.0)
            key.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2, -2, 3),
                                    Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 1)).GetInverse())
            route['spawn'] = [0, 0, 0.025, 0]
        light = UsdLux.DomeLight.Define(stage, '/World/AMRAmbient')
        light.CreateIntensityAttr(1200.0 if a.inspection else 350.0)
        root = "/World/Robot"
        robot_prim = UsdGeom.Xform.Define(stage, root)
        robot_prim.GetPrim().GetReferences().AddReference(str(PROJECT / 'assets/robot' / f'{a.robot}.usda'))
        robot_prim.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(*route['spawn'][:3]))
        robot_prim.AddRotateZOp().Set(float(route['spawn'][3]))
        base_path = f"{root}/base_link"
        required = [root, base_path, f"{root}/left_drive", f"{root}/right_drive"]
        required += [f"{base_path}/{s}_camera_link/camera" for s in ("front", "rear")]
        required += [f"{base_path}/{s}_lidar_link" for s in ("front_right", "rear_left")]
        absent = [p for p in required if not stage.GetPrimAtPath(p).IsValid()]
        if absent:
            raise RuntimeError(f"World has missing required robot prims: {absent}")
        if not stage.GetPrimAtPath(base_path).HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("base_link must be a dynamic rigid body")
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
        robot = SingleArticulation(prim_path=base_path if a.robot == "locomanipulator" else root, name="photo_amr", reset_xform_properties=False)
        from isaacsim.sensors.physics import IMUSensor
        imu = IMUSensor(base_path + '/imu_link', frequency=cfg['imu']['hz'], translation=np.zeros(3))
        if a.robot == 'locomanipulator' and not a.inspection:
            from isaac_runtime.manipulator import prepare_buttons
            prepare_buttons(stage)
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
        if len(all_joint_names) != (19 if a.robot == "locomanipulator" else 10):
            raise RuntimeError(f"Unexpected robot DOFs, received {all_joint_names}")

        d = cfg["drive"]
        limiter = DriveLimiter(radius=d["radius_m"], track=d["track_m"],
            max_speed=d["max_speed_mps"], max_accel=d["max_accel_mps2"],
            max_yaw_rate=d["max_yaw_rate_rps"], max_yaw_accel=d["max_yaw_accel_rps2"],
            timeout=d["command_timeout_s"])
        rclpy.init(args=[])
        node = rclpy.create_node("aprl_robot_sim")
        data_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pubs = {
            "clock": node.create_publisher(Clock, "/clock", reliable_qos),
            "odom": node.create_publisher(Odometry, "/ground_truth/odom", reliable_qos),
            "joint_states": node.create_publisher(JointState, "/joint_states", reliable_qos),
            "tf": node.create_publisher(TFMessage, "/tf", reliable_qos),
            "tf_static": node.create_publisher(TFMessage, "/tf_static", static_qos),
            "robot_description": node.create_publisher(String, "/robot_description", static_qos),
        }
        description = (PROJECT / "src/aprl_robot_sim/urdf" / f"{a.robot}.urdf").read_text()
        pubs["robot_description"].publish(String(data=description))
        current_time = [float(sim.current_time)]
        ready = [False]

        def command(msg):
            # Reject commands during warm-up, so a stale teleop cannot launch the robot.
            if not ready[0] or a.inspection:
                return
            limiter.command(float(msg.linear.x), float(msg.angular.z),
                            current_time[0], time.monotonic())

        subscription = node.create_subscription(Twist, "/cmd_vel", command, reliable_qos)

        def stamp(seconds):
            ns = max(0, round(float(seconds) * 1e9))
            return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

        from isaac_runtime.sensors import Sensors
        sensors = Sensors(stage, node, cfg, data_qos, stamp, imu)
        sensors.initialize()
        elevator_pub = node.create_publisher(String, '/elevator/state', reliable_qos)
        if not a.inspection:
            from isaac_runtime.elevator_ui import TestbedRuntime
            elevators = TestbedRuntime(ui_enabled=not a.headless, selection_presses=not a.headless)

        if a.robot == 'locomanipulator':
            from isaac_runtime.manipulator import Manipulator
            arm = Manipulator(stage, robot, node, cfg['manipulator'], elevators)

        def elevator_command(msg):
            try:
                request = json.loads(msg.data)
                if not isinstance(request, dict) or elevators is None:
                    raise ValueError('Expected an elevator command object in a multi-floor scene')
                eid, action = request['id'], request['action']
                floor = request.get('floor', 1)
                if type(floor) is not int or eid not in elevators.scene.rigs:
                    raise ValueError('Invalid floor or elevator ID')
                elevators.press(eid, action, floor - 1, request.get('direction', 'up'))
            except (ValueError, KeyError, TypeError) as exc:
                node.get_logger().warning(f'Rejected elevator command: {exc}')

        elevator_subscription = node.create_subscription(String, '/elevator/command', elevator_command, reliable_qos)

        def publish_elevators(seconds):
            state = {'scene': a.scene, 'robot': a.robot, 'ready': ready[0], 'sim_time': seconds, 'elevators': {}}
            if elevators is not None:
                state['elevators'] = {eid: rig.controller.snapshot() for eid, rig in elevators.scene.rigs.items()}
            elevator_pub.publish(String(data=json.dumps(state)))
            if arm:
                arm.publish()

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
        camera_specs = {side: (f'{base_path}/{side}_camera_link', camera_cfg) for side in ('front', 'rear')}
        if a.robot == 'locomanipulator':
            wrist = next(p for p in stage.Traverse() if p.GetName() == 'wrist_camera_link')
            camera_specs['wrist'] = (str(wrist.GetPath()), cfg['manipulator']['wrist_camera'])
        report["camera_frames"] = {"front": 0, "rear": 0}
        report["camera_valid_depth_frames"] = {"front": 0, "rear": 0}
        for side, (path, camera_settings) in camera_specs.items():
            link = f"{side}_camera_link"
            parent = base_path if side != 'wrist' else str(stage.GetPrimAtPath(path).GetParent().GetPath())
            static_transforms.append(transform(stage.GetPrimAtPath(parent).GetName(), link, relative(path, parent, cache), 0))
            # USD camera: -Z forward, +Y up. ROS optical: +Z forward, +Y down.
            usd_to_optical = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
            optical_in_link = usd_to_optical * relative(f"{path}/camera", path, cache)
            static_transforms.append(transform(link, f"{side}_camera_optical_frame", optical_in_link, 0))
            cam = Camera(prim_path=f"{path}/camera", name=f"{side}_rgbd",
                         frequency=int(camera_settings["hz"]),
                         resolution=(int(camera_settings["width"]), int(camera_settings["height"])))
            cam.initialize()
            cam.add_distance_to_image_plane_to_frame()
            # Pinhole intrinsics already authored in USD; publish their actual values.
            cam.set_clipping_range(0.01, 20.0)
            cameras.append({"name": side, "camera": cam, "config": camera_settings, "last_frame": None,
                            "last_publish_time": -math.inf, "count": 0,
                            "valid_depth_frames": 0, "textured_rgb_frames": 0})
            topic = f'/{side}_camera/depth/points'
            pubs[topic] = node.create_publisher(PointCloud2, topic, data_qos)
            for channel in ("color", "depth"):
                prefix = f"/{side}_camera/{channel}"
                pubs[prefix + "/image_raw"] = node.create_publisher(Image, prefix + "/image_raw", data_qos)
                pubs[prefix + "/camera_info"] = node.create_publisher(CameraInfo, prefix + "/camera_info", data_qos)
        for side in ("front_right", "rear_left"):
            link = f"{side}_lidar_link"
            static_transforms.append(transform("base_link", link, relative(f"{base_path}/{link}", base_path, cache), 0))
            topic = f"/{side}_lidar/scan"
            pubs[topic] = node.create_publisher(LaserScan, topic, data_qos)
        for link in ('flash_lidar_link', 'imu_link'):
            static_transforms.append(transform('base_link', link, relative(f'{base_path}/{link}', base_path, cache), 0))
        pubs["tf_static"].publish(TFMessage(transforms=static_transforms))

        moving_links = []
        for prim in Usd.PrimRange(robot_prim.GetPrim()):
            if prim.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(prim)
                parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
                if parents and children:
                    pp, cp = str(parents[0]), str(children[0])
                    moving_links.append((stage.GetPrimAtPath(pp).GetName(), stage.GetPrimAtPath(cp).GetName(), pp, cp))
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
            if seconds > 2 and (pos[2] < 0.06 or pos[2] > 13.0):
                raise RuntimeError(f"Robot height out of bounds: base_link z={pos[2]:.4f} m")
            z_values.append(float(pos[2]))
            if len(z_values) > 1200:
                del z_values[:600]
            odom_matrix = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)))
            odom_matrix.SetTranslateOnly(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2] - cfg["base_link_z_m"])))
            base_relative = bm * odom_matrix.GetInverse()
            tf = [transform("sim_world", "base_footprint", odom_matrix, seconds),
                  transform("base_footprint", "base_link", base_relative, seconds)]
            for parent, child, pp, cp in moving_links:
                tf.append(transform(parent, child, relative(cp, pp, c), seconds))
            pubs["tf"].publish(TFMessage(transforms=tf))
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp(seconds), "sim_world"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, pos)
            orientation = bm.ExtractRotationQuat().GetNormalized()
            odom.pose.pose.orientation.w = float(orientation.GetReal())
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = map(float, orientation.GetImaginary())
            old = previous_pose[0]
            if old and seconds > old[0]:
                delta_t = seconds - old[0]
                vel = (pos - old[1]) / delta_t
                odom.twist.twist.linear.z = float(vel[2])
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
            sensors.publish_wheels(seconds, np.asarray(joints.position)[indices])
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

        def camera_info(camera, seconds, side, camera_settings):
            k = np.asarray(camera.get_intrinsics_matrix(), dtype=np.float64)
            info = CameraInfo()
            info.header.stamp = stamp(seconds)
            info.header.frame_id = f"{side}_camera_optical_frame"
            info.width, info.height = int(camera_settings["width"]), int(camera_settings["height"])
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = k.reshape(-1).tolist()
            info.r = np.eye(3).reshape(-1).tolist()
            info.p = np.column_stack((k, np.zeros(3))).reshape(-1).tolist()
            return info

        def publish_cameras(seconds):
            from isaac_runtime.rgbd import pointcloud
            for entry in cameras:
                cam, side = entry["camera"], entry["name"]
                settings = entry["config"]
                width, height = int(settings["width"]), int(settings["height"])
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
                valid = np.isfinite(depth) & (depth >= settings["min_depth_m"]) & (depth <= settings["max_depth_m"])
                depth = depth.copy()
                depth[~valid] = np.nan
                info = camera_info(cam, timestamp, side, settings)
                pubs[f"/{side}_camera/depth/points"].publish(pointcloud(depth, rgb, info, settings["pointcloud_stride"]))
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

        if not a.headless or a.capture_orbit:
            from isaac_runtime.views import RobotViews
            views = RobotViews(stage, tall=a.robot == "locomanipulator")
            if a.inspection:
                views.select('orbit')
        capture = None
        if a.capture_orbit:
            a.capture_orbit.mkdir(parents=True, exist_ok=True)
            capture = Camera(prim_path='/World/RobotView', name='turntable', resolution=(960, 640), frequency=rhz)
            capture.initialize()
        capture_count = 0
        report.update({"phase": "running", "robot": a.robot, "scene": a.scene, "python_version": sys.version.split()[0],
                       "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
                       "initial_caster_yaw_deg": a.caster_yaw_deg, "joint_names": all_joint_names,
                       "odom_source": "wheel joint encoder integration; separate /ground_truth/odom",
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
        startup_wall, ready_time = start_wall, None
        while app.is_running() and rclpy.ok() and not stop_requested[0]:
            if not sim.is_playing():
                if sim.is_stopped():
                    print("Timeline stopped. Relaunch scripts/sim for a full reset.", flush=True)
                    break
                app.update()
                rclpy.spin_once(node, timeout_sec=0.0)
                # Clear stale commands across GUI pause/resume.
                limiter.requested_linear = limiter.requested_angular = 0.0
                limiter.linear = limiter.angular = 0.0
                limiter.last_sim = limiter.last_wall = -math.inf
                start_wall = time.monotonic() - (float(sim.current_time) - start_sim)
                continue
            current_time[0] = float(sim.current_time)
            rclpy.spin_once(node, timeout_sec=0.0)
            if arm:
                if arm.busy:
                    limiter.requested_linear = limiter.requested_angular = 0.0
                arm.step(dt, current_time[0])
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
            sensors.publish_imu()
            if frame_idx % state_every == 0:
                publish_state(seconds)
            if frame_idx % scan_every == 0:
                publish_scans(seconds)
                publish_elevators(seconds)
            if frame_idx % render_every == 0:
                if views:
                    views.update(seconds)
                sim.render()
                publish_cameras(seconds)
                sensors.publish_cloud()
                if capture and seconds > 3.0 and capture_count < 120:
                    from PIL import Image as PILImage
                    rgba = capture.get_rgba()
                    if rgba is not None and rgba.size and frame_idx % (phz // 10) == 0:
                        PILImage.fromarray(rgba[:, :, :3].astype(np.uint8)).save(a.capture_orbit / f'{capture_count:04d}.png')
                        capture_count += 1
            report["sensor_frames"] = dict(sensors.counts)
            elapsed = seconds - start_sim
            if not ready[0]:
                sensors_ready = all(e["count"] > 0 and (e["name"] == "wrist" or
                                    (e["textured_rgb_frames"] > 0 and e["valid_depth_frames"] > 0)) for e in cameras)
                sensors_ready &= a.inspection or all(finite_scan_counts[s] > 0 for s in scan_counts)
                sensors_ready &= sensors.counts["imu"] > 0 and sensors.counts["flash"] > 0
                if elapsed >= 2.0 and sensors_ready and physics_health.ready:
                    ready[0] = True
                    ready_time = seconds
                    report["startup_checks_passed"] = True
                    report["settled_base_z_m"] = report["end_base_position_m"][2]
                    print("AMR_READY: RGBD, 2D/FLASH lidar, IMU, wheel odometry and elevators are live. /cmd_vel enabled.", flush=True)
                elif time.monotonic() - startup_wall > a.startup_timeout:
                    raise RuntimeError(f"Startup check failed: sensors_ready={sensors_ready}, "
                                       f"physics_stable={physics_health.ready}, camera_frames={report['camera_frames']}")
            if time.monotonic() - last_status_wall > 10:
                print(f"AMR sim={elapsed:.1f}s ready={ready[0]} cmd_count={limiter.accepted} "
                      f"camera_frames={[e['count'] for e in cameras]} scans={scan_counts} "
                      f"tilt_deg={report['physics_latest']['tilt_deg']:.3f}", flush=True)
                if a.physics_trace:
                    print('PHYSICS', report['end_base_position_m'], report['end_yaw_rad'],
                          'targets', left, right, 'joints', robot.get_joint_velocities().tolist(), flush=True)
                last_status_wall = time.monotonic()
            if a.duration and ready_time is not None and seconds - ready_time >= a.duration:
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
        # The external ROS route checks commanded travel and physical elevator riding.
        report["runtime_startup_passed"] = True
    except BaseException as exc:
        report.update({"phase": "failed", "error": f"{type(exc).__name__}: {exc}"})
        traceback.print_exc()
        raise
    finally:
        if arm is not None:
            report['arm_contacts'] = arm.events
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"Runtime report: {a.report}", flush=True)
        if arm is not None:
            arm.close()
        if sensors is not None:
            sensors.close()
        if views is not None:
            views.close()
        if elevators is not None:
            elevators.destroy()
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
              "test_scope": "live startup and stream checks; travel verified by the external ROS route"}
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
