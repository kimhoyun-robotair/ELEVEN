"""Native PhysX IMU, simultaneous RTX FLASH returns, and wheel encoder odometry."""
import math

import numpy as np
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from isaacsim.sensors.rtx import LidarRtx


class Sensors:
    def __init__(self, stage, node, cfg, qos, stamp, imu, robot='amr'):
        self.cfg, self.stamp = cfg, stamp
        base = '/World/Robot/base_link'
        self.imu = imu
        self.imu_pub = node.create_publisher(Imu, '/mid360/imu' if robot == 'scout' else '/imu/data', qos)
        self.odom_pub = node.create_publisher(Odometry, '/odom', 10)
        self.cloud_kind = 'mid360' if robot == 'scout' else 'flash'
        self.last_imu = self.last_cloud = -math.inf
        self.counts = {'imu': 0, self.cloud_kind: 0, 'wheel_odom': 0}
        self.wheel_pose = np.zeros(3)
        self.last_wheels = None
        self.last_wheel_time = None
        self.lidar = self.mid360 = None
        if robot == 'scout':
            from isaac_runtime.scout import Mid360Lidar
            self.mid360 = Mid360Lidar(cfg['lidar'], node, qos, stamp)
            return
        self.cloud_pub = node.create_publisher(PointCloud2, '/mlx/pointcloud', qos)
        f = cfg['flash_lidar']
        prim = stage.DefinePrim(base + '/flash_lidar_link/sensor', 'OmniLidar')
        prim.ApplyAPI('OmniSensorGenericLidarCoreAPI')
        count = f['width'] * f['height']
        az, el = np.meshgrid(np.linspace(-f['hfov_deg']/2, f['hfov_deg']/2, f['width']),
                             np.linspace(-f['vfov_deg']/2, f['vfov_deg']/2, f['height']))
        values = {
            'scanType': 'SOLID_STATE', 'numberOfEmitters': count, 'numberOfChannels': count,
            'rangeCount': 1, 'rangesMinM': [f['range_min_m']], 'rangesMaxM': [f['range_max_m']],
            'scanRateBaseHz': f['hz'], 'reportRateBaseHz': f['hz'],
            'nearRangeM': f['range_min_m'], 'farRangeM': f['range_max_m'],
            'maxReturns': 1, 'numLines': f['height'], 'numRaysPerLine': [f['width']] * f['height'],
            'rayType': 'IDEALIZED', 'azimuthErrorStd': 0.0, 'elevationErrorStd': 0.0,
            'rangeAccuracyM': 0.0, 'outputFrameOfReference': 'SENSOR', 'auxOutputType': 'BASIC',
            'emitterState:s001:azimuthDeg': az.ravel().tolist(),
            'emitterState:s001:elevationDeg': el.ravel().tolist(),
            'emitterState:s001:fireTimeNs': [0] * count,
            'emitterState:s001:channelId': list(range(1, count + 1)),
            'emitterState:s001:bank': np.repeat(np.arange(f['height']), f['width']).tolist(),
            'emitterState:s001:rangeId': [0] * count,
        }
        for key, value in values.items():
            attr = prim.GetAttribute('omni:sensor:Core:' + key)
            if not attr:
                raise RuntimeError(f'Isaac 5.1 lidar schema missing {key}')
            attr.Set(value)
        prim.GetAttribute('omni:sensor:tickRate').Set(float(f['hz']))
        self.lidar = LidarRtx(str(prim.GetPath()), name='amr_flash_lidar')
        self.annotator = 'IsaacCreateRTXLidarScanBuffer'
        self.lidar.attach_annotator(self.annotator, transformPoints=False, outputIntensity=True)

    def initialize(self):
        self.imu.initialize()
        if self.lidar is not None:
            self.lidar.initialize()

    def publish_imu(self):
        frame = self.imu.get_current_frame(read_gravity=True)
        seconds = float(frame['time'])
        if seconds <= self.last_imu or seconds <= 0:
            return
        self.last_imu = seconds
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = self.stamp(seconds), 'imu_link'
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, frame['lin_acc'])
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, frame['ang_vel'])
        # No magnetometer / absolute attitude is supplied to the estimator.
        msg.orientation_covariance[0] = -1.0
        msg.linear_acceleration_covariance = [0.0004, 0., 0., 0., 0.0004, 0., 0., 0., 0.0004]
        msg.angular_velocity_covariance = [0.000001, 0., 0., 0., 0.000001, 0., 0., 0., 0.000001]
        self.imu_pub.publish(msg)
        self.counts['imu'] += 1

    def publish_cloud(self, seconds):
        if self.mid360 is not None:
            self.counts['mid360'] += int(self.mid360.publish(seconds))
            return
        if self.lidar is None:
            raise RuntimeError('No RTX lidar configured')
        frame = self.lidar.get_current_frame()
        seconds = float(frame['rendering_time'])
        if seconds <= 0 or seconds - self.last_cloud < 1 / self.cfg['flash_lidar']['hz'] - 1e-5:
            return
        scan = frame.get(self.annotator, {})
        points = np.asarray(scan.get('data', []), dtype=np.float32)
        if points.size == 0:
            return
        points = points.reshape(-1, 3)
        ranges = np.linalg.norm(points, axis=1)
        f = self.cfg['flash_lidar']
        valid = np.isfinite(points).all(axis=1) & (ranges >= f['range_min_m']) & (ranges <= f['range_max_m'])
        points = points[valid]
        if len(points) == 0:
            return
        # Every emitter fires at the same instant. t is a relative nanosecond offset.
        data = np.zeros(len(points), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('t','<u4')])
        for i, key in enumerate(('x', 'y', 'z')):
            data[key] = points[:, i]
        intensity = np.asarray(scan.get('intensity', []), dtype=np.float32)
        if intensity.size == valid.size:
            data['intensity'] = intensity[valid]
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = self.stamp(seconds), 'flash_lidar_link'
        msg.height, msg.width, msg.is_dense = 1, len(data), True
        msg.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32 if k != 't' else PointField.UINT32, count=1)
                      for i, k in enumerate(data.dtype.names)]
        msg.point_step, msg.row_step, msg.data = 20, len(data)*20, data.tobytes()
        self.cloud_pub.publish(msg)
        self.last_cloud = seconds
        self.counts['flash'] += 1

    def publish_wheels(self, seconds, joints):
        if self.last_wheels is None:
            self.last_wheels, self.last_wheel_time = np.array(joints), seconds
            return
        delta = np.array(joints) - self.last_wheels
        dt = seconds - self.last_wheel_time
        self.last_wheels, self.last_wheel_time = np.array(joints), seconds
        if dt <= 0:
            return
        d = self.cfg['drive']
        distance = d['radius_m'] * float(delta.sum()) / 2
        angle = d['radius_m'] * float(delta[1] - delta[0]) / d['track_m']
        yaw = float(self.wheel_pose[2])
        scale = math.sin(angle / 2) / (angle / 2) if abs(angle) > 1e-9 else 1.0
        self.wheel_pose += [distance*scale*math.cos(yaw+angle/2), distance*scale*math.sin(yaw+angle/2), angle]
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = self.stamp(seconds), 'wheel_odom', 'wheel_base'
        msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, self.wheel_pose[:2])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(self.wheel_pose[2]/2), math.cos(self.wheel_pose[2]/2)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = distance/dt, angle/dt
        for index in (0, 7, 35):
            msg.pose.covariance[index] = 0.01 + 0.001 * seconds
            msg.twist.covariance[index] = 0.01
        for index in (14, 21, 28):
            msg.pose.covariance[index] = msg.twist.covariance[index] = 1e6
        self.odom_pub.publish(msg)
        self.counts['wheel_odom'] += 1

    def close(self):
        if self.lidar is not None:
            self.lidar.detach_all_annotators()
        if self.mid360 is not None:
            self.mid360.close()
