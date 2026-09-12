"""Scout Twin configuration and native Isaac 5.1 sensors for the APRL runtime."""
import math
from typing import TypedDict


class CameraSpec(TypedDict):
    path: str
    sensor: str
    config: dict[str, float | int]
    channels: tuple[str, ...]
    topic: str
    frame: str


def runtime_config(source):
    base, camera = source['base'], source['camera_spec']
    return {**source,
            'base_link_z_m': base['wheel_radius'],
            'drive': {
                'radius_m': base['wheel_radius'], 'width_m': base['wheel_width'],
                'track_m': base['track'], 'max_speed_mps': base['max_linear_speed'],
                'max_yaw_rate_rps': base['max_angular_speed'],
                'max_accel_mps2': 0.6, 'max_yaw_accel_rps2': 1.5,
                'command_timeout_s': base['command_timeout']},
            'imu': {'hz': source['simulation']['imu_hz']},
            'camera': {
                'width': camera['resolution'][0], 'height': camera['resolution'][1],
                'hz': camera['rate_hz'], 'min_depth_m': camera['min_depth'],
                'max_depth_m': camera['max_depth'], 'clip_far_m': camera['clip_far']}}


def camera_specs(base_path, cfg) -> dict[str, CameraSpec]:
    return {f"{entry['name']}/{channel}": {
                'path': f"{base_path}/{entry['name']}_link", 'sensor': f'{channel}_sensor',
                'config': cfg['camera'], 'channels': (channel,),
                'topic': f"/{entry['name']}",
                'frame': f"{entry['name']}_{channel}_optical_frame"}
            for entry in cfg['cameras'] for channel in ('color', 'depth')}


def configure_camera(camera, stage, channel, cfg):
    import numpy as np
    from pxr import UsdGeom

    spec = cfg['camera_spec']
    camera.set_local_pose(translation=np.array(spec['optical_xyz']),
                          orientation=np.array([1, 0, 0, 0]), camera_axes='world')
    usd = UsdGeom.Camera(stage.GetPrimAtPath(camera.prim_path))
    hfov = spec['rgb_hfov_deg'] if channel == 'color' else spec['depth_hfov_deg']
    focal = 12.0
    aperture = 2 * focal * math.tan(math.radians(hfov / 2))
    width, height = spec['resolution']
    usd.GetFocalLengthAttr().Set(focal)
    usd.GetHorizontalApertureAttr().Set(aperture)
    usd.GetVerticalApertureAttr().Set(aperture * height / width)
    usd.GetFStopAttr().Set(0.0)


class Mid360Lidar:
    """RTX rotary lattice from Scout Twin: geometric FOV/rate approximation."""

    def __init__(self, cfg, node, qos, stamp):
        import carb
        import numpy as np
        import omni.replicator.core as rep
        from sensor_msgs.msg import PointCloud2

        self.cfg, self.stamp = cfg, stamp
        self.publisher = node.create_publisher(PointCloud2, '/mid360/points', qos)
        self.last_scan = None
        self.last_publish = -math.inf
        carb.settings.get_settings().set_bool('/app/sensors/nv/lidar/outputBufferOnGPU', True)
        count = 100
        attrs = {
            'scanType': 'ROTARY', 'scanRateBaseHz': int(cfg['rate_hz']),
            'reportRateBaseHz': int(cfg['points_per_second'] / count),
            'numberOfEmitters': count, 'numberOfChannels': count, 'maxReturns': 1,
            'nearRangeM': float(cfg['min_range']), 'farRangeM': float(cfg['max_range']),
            'rangeOffsetM': 0.065, 'rangeAccuracyM': 0.0,
            'azimuthErrorStd': 0.0, 'elevationErrorStd': 0.0,
            'rotationDirection': 'CCW', 'outputFrameOfReference': 'SENSOR',
            'elementsCoordsType': 'SPHERICAL', 'auxOutputType': 'BASIC',
            'skipDroppingInvalidPoints': True,
            'emitterState:s001:azimuthDeg': [0.0] * count,
            'emitterState:s001:elevationDeg': np.linspace(cfg['elevation_min_deg'], cfg['elevation_max_deg'], count).tolist(),
            'emitterState:s001:fireTimeNs': [0] * count,
            'emitterState:s001:channelId': list(range(1, count + 1)),
        }
        parent = '/World/Robot/base_link/mid360_link'
        prim = rep.functional.create.omni_lidar(
            name='rtx_sensor', parent=parent,
            **{'omni:sensor:Core:' + key: value for key, value in attrs.items()})
        if not prim.IsValid() or str(prim.GetPath()) != parent + '/rtx_sensor':
            raise RuntimeError('MID-360 RTX sensor creation returned an unexpected prim')
        self.product = rep.create.render_product(prim.GetPath(), resolution=(1, 1))
        self.annotator = rep.AnnotatorRegistry.get_annotator('IsaacCreateRTXLidarScanBuffer', device='cpu')
        self.annotator.initialize(outputTimestamp=True, outputIntensity=True, transformPoints=False)
        self.annotator.attach([self.product.path])

    def publish(self, seconds):
        import numpy as np
        from sensor_msgs.msg import PointCloud2, PointField

        if seconds <= 0 or seconds - self.last_publish < 1 / self.cfg['rate_hz'] - 1e-5:
            return False
        scan = self.annotator.get_data()
        if not isinstance(scan, dict):
            return False
        if isinstance(scan.get('info'), dict):
            scan = {**scan['info'], **scan}
        points = np.asarray(scan.get('data', []), dtype=np.float32).reshape(-1, 3)
        timestamps = np.asarray(scan.get('timestamp', []))
        if not points.size or not timestamps.size:
            return False
        newest = int(timestamps.max())
        if newest == self.last_scan:
            return False
        intensity = np.asarray(scan.get('intensity', []), dtype=np.float32).reshape(-1)
        if len(intensity) != len(points):
            raise RuntimeError('MID-360 RTX intensity/point count mismatch')
        ranges = np.linalg.norm(points, axis=1)
        valid = np.isfinite(points).all(axis=1) & np.isfinite(intensity)
        valid &= (ranges >= self.cfg['min_range']) & (ranges <= self.cfg['max_range'])
        if not valid.any():
            return False
        data = np.column_stack((points[valid], intensity[valid])).astype('<f4')
        msg = PointCloud2()
        # The RTX accumulator's internal timestamps are not ROS simulation time.
        msg.header.stamp, msg.header.frame_id = self.stamp(seconds), 'mid360_link'
        msg.height, msg.width, msg.is_dense = 1, len(data), True
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        msg.point_step, msg.row_step, msg.data = 16, len(data) * 16, data.tobytes()
        self.publisher.publish(msg)
        self.last_scan, self.last_publish = newest, seconds
        return True

    def close(self):
        self.annotator.detach([self.product.path])
        self.product.destroy()
