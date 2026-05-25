from __future__ import annotations

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField


def pointcloud2_to_xyzi(msg: PointCloud2) -> np.ndarray:
    fast_points = _pointcloud2_to_xyzi_frombuffer(msg)
    if fast_points is not None:
        return fast_points

    field_names = {field.name for field in msg.fields}
    has_intensity = "intensity" in field_names
    requested_fields = ["x", "y", "z", "intensity"] if has_intensity else ["x", "y", "z"]
    points = pc2.read_points_numpy(msg, field_names=requested_fields, skip_nans=True)

    if points.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    points = np.asarray(points, dtype=np.float32)
    if points.ndim == 1:
        points = points.reshape(1, -1)
    if not has_intensity:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        points = np.concatenate([points, intensity], axis=1)
    return np.ascontiguousarray(points[:, :4], dtype=np.float32)


def msg_timestamp_sec(msg: PointCloud2) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)


def _pointcloud2_to_xyzi_frombuffer(msg: PointCloud2) -> np.ndarray | None:
    if msg.is_bigendian:
        return None
    if msg.height != 1:
        return None
    if msg.row_step != msg.point_step * msg.width:
        return None

    fields = {field.name: field for field in msg.fields}
    required = ("x", "y", "z")
    if any(name not in fields for name in required):
        return None
    if any(fields[name].datatype != PointField.FLOAT32 for name in required):
        return None
    has_intensity = "intensity" in fields and fields["intensity"].datatype == PointField.FLOAT32

    dtype_fields = [
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
    ]
    offsets = [
        fields["x"].offset,
        fields["y"].offset,
        fields["z"].offset,
    ]
    if has_intensity:
        dtype_fields.append(("intensity", np.float32))
        offsets.append(fields["intensity"].offset)
    dtype = np.dtype(
        {
            "names": [name for name, _ in dtype_fields],
            "formats": [fmt for _, fmt in dtype_fields],
            "offsets": offsets,
            "itemsize": msg.point_step,
        }
    )
    structured = np.frombuffer(msg.data, dtype=dtype, count=msg.width)
    points = np.empty((structured.shape[0], 4), dtype=np.float32)
    points[:, 0] = structured["x"]
    points[:, 1] = structured["y"]
    points[:, 2] = structured["z"]
    finite_mask = np.isfinite(points[:, :3]).all(axis=1)
    if has_intensity:
        points[:, 3] = structured["intensity"].astype(np.float32, copy=False)
        finite_mask &= np.isfinite(points[:, 3])
    else:
        points[:, 3] = 0.0

    return np.ascontiguousarray(points[finite_mask], dtype=np.float32)

