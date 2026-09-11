"""Verify live Isaac/ROS streams; no mocks or generated test messages."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, Image, CameraInfo, JointState, LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


class StreamCheck(Node):
    def __init__(self, lio=False, robot="amr"):
        super().__init__('aprl_stream_check')
        self.counts, self.stamps, self.first_stamp, self.details = {}, {}, {}, {}
        self.errors, self.edges = set(), set()
        self.ready = False
        self.robot = robot
        self.frames, self.matched = {}, {}
        topics = {'/mlx/pointcloud':PointCloud2, '/imu/data':Imu, '/odom':Odometry,
                  '/ground_truth/odom':Odometry, '/clock':Clock, '/joint_states':JointState,
                  '/tf':TFMessage, '/tf_static':TFMessage, '/elevator/state':String,
                  '/robot_description':String}
        if robot == 'locomanipulator':
            topics['/arm/state'] = String
        for side in (('front','rear','wrist') if robot == 'locomanipulator' else ('front','rear')):
            topics[f'/{side}_camera/depth/points'] = PointCloud2
            for channel in ('color','depth'):
                topics[f'/{side}_camera/{channel}/image_raw'] = Image
                topics[f'/{side}_camera/{channel}/camera_info'] = CameraInfo
        for side in ('front_right','rear_left'):
            topics[f'/{side}_lidar/scan'] = LaserScan
        if lio:
            topics['/LIO/odom_imu'] = Odometry
            topics['/LIO/clouds_lidar'] = PointCloud2
        for topic, kind in topics.items():
            self.counts[topic] = 0
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ('/tf_static', '/robot_description') else qos_profile_sensor_data
            self.create_subscription(kind, topic, lambda msg,t=topic:self.receive(t,msg), qos)

    def receive(self, topic, msg):
        self.counts[topic] += 1
        stamp = msg.clock if isinstance(msg, Clock) else getattr(getattr(msg,'header',None),'stamp',None)
        if stamp is not None:
            seconds = stamp.sec + stamp.nanosec*1e-9
            if topic in self.stamps and seconds < self.stamps[topic]:
                self.errors.add(f'{topic}: backwards timestamp')
            self.first_stamp.setdefault(topic, seconds)
            self.stamps[topic] = seconds
        if isinstance(msg, PointCloud2):
            if msg.width*msg.height == 0 or len(msg.data) != msg.row_step*msg.height:
                self.errors.add(f'{topic}: invalid point buffer')
            if topic == '/mlx/pointcloud':
                expected = [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('t',16,6)]
                if [(f.name,f.offset,f.datatype) for f in msg.fields] != expected or msg.point_step != 20:
                    self.errors.add('FLASH: incompatible LIO field layout')
                    return
                data = np.frombuffer(msg.data, dtype=[('xyz','<f4',(3,)),('intensity','<f4'),('t','<u4')])
                xyz = data['xyz']
                if not np.isfinite(xyz).all() or np.any(data['t'] != 0):
                    self.errors.add('FLASH: nonfinite points or non-simultaneous returns')
                if np.any(xyz[:,0] <= 0) or np.max(np.abs(np.degrees(np.arctan2(xyz[:,1],xyz[:,0])))) > 60.1:
                    self.errors.add('FLASH: incorrect forward axis/FOV')
                self.details['flash'] = {'points':len(data),'range_m':[float(np.linalg.norm(xyz,axis=1).min()),float(np.linalg.norm(xyz,axis=1).max())]}
            elif '_camera/depth/points' in topic:
                layout = [(f.name, f.offset, f.datatype) for f in msg.fields]
                if layout != [('x',0,7),('y',4,7),('z',8,7),('rgb',12,7)] or msg.point_step != 16:
                    self.errors.add(f'{topic}: unexpected RGBD point layout')
                    return
                xyz = np.frombuffer(msg.data, dtype='<f4').reshape(msg.height, msg.width, 4)[..., :3]
                count = int(np.isfinite(xyz[..., 2]).sum())
                if count:
                    self.details[topic] = {'finite_points':count, 'size':[msg.width,msg.height], 'frame':msg.header.frame_id}
                self.sync_camera(topic.split('/')[1], 'points', msg)
        elif isinstance(msg, Imu):
            a, w = msg.linear_acceleration, msg.angular_velocity
            if not all(math.isfinite(v) for v in (a.x,a.y,a.z,w.x,w.y,w.z)):
                self.errors.add('IMU: nonfinite data')
            self.details['imu_acceleration'] = [a.x,a.y,a.z]
        elif isinstance(msg, JointState):
            if len(msg.name) != (19 if self.robot == 'locomanipulator' else 10) or not np.isfinite(msg.position).all():
                self.errors.add('Unexpected or invalid articulation state')
            self.details['joint_names'] = list(msg.name)
        elif isinstance(msg, Image):
            self.sync_camera(topic.split('/')[1], 'depth' if '/depth/' in topic else 'color', msg)
            if len(msg.data) != msg.height*msg.step or msg.width == 0:
                self.errors.add(f'{topic}: invalid image buffer')
            if '/depth/' in topic and msg.encoding == '32FC1':
                depth = np.frombuffer(msg.data,dtype='<f4')
                if np.isfinite(depth).any():
                    self.details[topic] = {'finite_depth':int(np.isfinite(depth).sum())}
        elif isinstance(msg, CameraInfo):
            self.sync_camera(topic.split('/')[1], 'info', msg)
            if msg.k[0] <= 0 or msg.k[4] <= 0:
                self.errors.add(f'{topic}: invalid focal length')
        elif isinstance(msg, LaserScan):
            if any(math.isfinite(v) for v in msg.ranges):
                self.details[topic] = {'finite_returns':sum(math.isfinite(v) for v in msg.ranges)}
        elif isinstance(msg, Odometry):
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
                self.errors.add(f'{topic}: invalid pose')
            self.details[topic] = {'position':[p.x,p.y,p.z], 'frame':msg.header.frame_id, 'child':msg.child_frame_id}
        elif isinstance(msg, TFMessage):
            self.edges.update((t.header.frame_id,t.child_frame_id) for t in msg.transforms)
            for tf in msg.transforms:
                if tf.child_frame_id == 'flash_lidar_link':
                    q, p = tf.transform.rotation, tf.transform.translation
                    upward = math.degrees(math.asin(max(-1, min(1, 2*(q.x*q.z-q.w*q.y)))))
                    if abs(upward-13) > .01 or math.dist((p.x,p.y,p.z),(.4,0,.022)) > .001:
                        self.errors.add('FLASH mount is not at the front recess with 13 degrees pitch up')
                    self.details['flash_mount'] = {'position_m':[p.x,p.y,p.z], 'pitch_up_deg':upward}
        elif isinstance(msg, String):
            if topic == '/robot_description':
                try:
                    robot = ET.fromstring(msg.data)
                    if robot.tag != 'robot' or robot.get('name') != self.robot:
                        self.errors.add('URDF does not describe the selected robot')
                    self.details['robot_description'] = {
                        'name': robot.get('name'), 'bytes': len(msg.data.encode()),
                        'links': [link.attrib['name'] for link in robot.findall('link')],
                        'movable_joints': [joint.attrib['name'] for joint in robot.findall('joint') if joint.get('type') != 'fixed'],
                        'mesh_resources': sorted({mesh.attrib['filename'] for mesh in robot.findall('.//mesh')})}
                except (ET.ParseError, KeyError) as exc:
                    self.errors.add(f'Invalid robot_description: {exc}')
            elif topic == '/elevator/state':
                state = json.loads(msg.data)
                self.ready = state['ready']
                if state.get('robot') != self.robot:
                    self.errors.add('Robot selection does not match the running simulator')
            else:
                self.details['arm'] = json.loads(msg.data)

    def sync_camera(self, side, kind, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        frames = self.frames.setdefault(side, {})
        frame = frames.setdefault(stamp, {})
        frame[kind] = msg
        while len(frames) > 8:
            frames.pop(next(iter(frames)))
        if not all(k in frame for k in ('points', 'depth', 'color', 'info')):
            return
        cloud, depth, color, info = (frame[k] for k in ('points', 'depth', 'color', 'info'))
        points = np.frombuffer(cloud.data, dtype='<f4').reshape(cloud.height, cloud.width, 4)
        stride = depth.width // cloud.width
        z = np.frombuffer(depth.data, dtype='<f4').reshape(depth.height, depth.width)[::stride, ::stride]
        valid = np.isfinite(z)
        if points.shape[:2] != z.shape or not np.allclose(points[...,2], z, atol=1e-6, equal_nan=True):
            self.errors.add(f'{side}: point Z does not match the simultaneous rendered depth')
        if valid.any():
            v, u = np.mgrid[:depth.height:stride, :depth.width:stride]
            projected_u = points[...,0][valid]/z[valid]*info.k[0]+info.k[2]
            projected_v = points[...,1][valid]/z[valid]*info.k[4]+info.k[5]
            if not np.allclose(projected_u, u[valid], atol=.001) or not np.allclose(projected_v, v[valid], atol=.001):
                self.errors.add(f'{side}: cloud does not reproject to its source pixels')
            rgb = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height,color.width,3)[::stride,::stride]
            packed = points[...,3].copy().view('<u4')
            actual_rgb = np.stack(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255), axis=-1)
            if not np.array_equal(actual_rgb, rgb):
                self.errors.add(f'{side}: cloud colors differ from the simultaneous image')
        self.matched[side] = self.matched.get(side, 0)+1
        del frames[stamp]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout',type=float,default=180)
    p.add_argument('--seconds',type=float,default=5,help='Observe this many simulation seconds')
    p.add_argument('--robot', choices=('amr','locomanipulator'), default='amr')
    p.add_argument('--lio',action='store_true')
    p.add_argument('--report',type=Path,default=Path('.runtime/logs/streams.json'))
    a = p.parse_args()
    rclpy.init()
    node = StreamCheck(a.lio, a.robot)
    deadline = time.monotonic()+a.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            advanced = all(node.stamps.get(t,0)-node.first_stamp.get(t,0) >= a.seconds for t in node.stamps)
            if node.ready and all(c >= (1 if t in ('/tf_static', '/robot_description') else 3) for t,c in node.counts.items()) and advanced:
                break
        else:
            node.errors.add('Timed out before all live streams advanced')
        description = node.details.get('robot_description', {})
        if set(description.get('movable_joints', ())) != set(node.details.get('joint_names', ())):
            node.errors.add('URDF joints differ from the live articulation')
        frames = {frame for edge in node.edges for frame in edge}
        missing = set(description.get('links', ())) - frames
        if missing:
            node.errors.add(f'URDF links missing from live TF: {sorted(missing)}')
        for edge in (('base_link','flash_lidar_link'),('base_link','imu_link'),('sim_world','base_footprint'),('base_footprint','base_link')):
            if edge not in node.edges:
                node.errors.add(f'Missing TF {edge}')
        for t in ('/front_camera/depth/image_raw','/rear_camera/depth/image_raw','/front_right_lidar/scan','/rear_left_lidar/scan'):
            if t not in node.details:
                node.errors.add(f'No finite return: {t}')
        for side in ('front_camera','rear_camera'):
            if not node.matched.get(side):
                node.errors.add(f'No simultaneous depth/color/cloud validation: {side}')
        report = {'camera_matches':node.matched, 'passed':not node.errors,'counts':node.counts,'errors':sorted(node.errors), 'details':node.details}
        a.report.parent.mkdir(parents=True,exist_ok=True)
        a.report.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
