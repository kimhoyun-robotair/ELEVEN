"""Organized, colored optical-frame points from rendered metric Z depth."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def pointcloud(depth, rgb, info, stride=2):
    z = depth[::stride, ::stride]
    v, u = np.mgrid[:depth.shape[0]:stride, :depth.shape[1]:stride]
    points = np.empty(z.shape, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    points['x'] = (u-info.k[2])*z/info.k[0]
    points['y'] = (v-info.k[5])*z/info.k[4]
    points['z'] = z
    colors = rgb[::stride, ::stride].astype(np.uint32)
    points['rgb'] = (colors[..., 0] << 16) | (colors[..., 1] << 8) | colors[..., 2]
    msg = PointCloud2()
    msg.header = info.header
    msg.height, msg.width = z.shape
    msg.is_dense = bool(np.isfinite(z).all())
    msg.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.point_step, msg.row_step = 16, msg.width*16
    msg.data = points.tobytes()
    return msg
