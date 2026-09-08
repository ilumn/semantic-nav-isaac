from __future__ import annotations

import struct
from typing import Iterable

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


_POINT_STRUCT = struct.Struct("<fffI")


def pack_rgb_uint32(r: int, g: int, b: int) -> int:
    return (
        (max(0, min(255, int(r))) << 16)
        | (max(0, min(255, int(g))) << 8)
        | max(0, min(255, int(b)))
    )


def build_xyzrgb_cloud(
    *,
    header: Header,
    points: Iterable[tuple[float, float, float, int]],
) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.is_bigendian = False
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    cloud.point_step = _POINT_STRUCT.size

    data = bytearray()
    point_count = 0
    for x, y, z, rgb in points:
        data.extend(_POINT_STRUCT.pack(float(x), float(y), float(z), int(rgb)))
        point_count += 1

    cloud.width = point_count
    cloud.row_step = cloud.point_step * point_count
    cloud.is_dense = False
    cloud.data = bytes(data)
    return cloud
