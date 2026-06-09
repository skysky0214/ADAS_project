from __future__ import annotations

import numpy as np
import open3d as o3d
from typing import List

from app.core.domain_types import DetectedObject, FrameInput
from app.perception.base import PerceptionModel

class ClusteringPerceptionModel(PerceptionModel):
    """
    Heuristic clustering-based pedestrian detector using RANSAC and DBSCAN.
    Does not require deep learning models or OpenPCDet.
    """

    def __init__(
        self,
        roi_x_min: float = 2.5,
        roi_x_max: float = 15.0,
        roi_y_min: float = -1.1,
        roi_y_max: float = 1.1,
        roi_z_min: float = -1.4,
        roi_z_max: float = 1.0,
        voxel_size: float = 0.08,
        plane_distance_threshold: float = 0.12,
        plane_ransac_n: int = 3,
        plane_num_iterations: int = 200,
        cluster_eps: float = 0.5,
        cluster_min_points: int = 12,
        human_height_min: float = 1.0,
        human_height_max: float = 2.0,
        human_width_max: float = 1.0,
        human_length_max: float = 1.3,
    ):
        self.roi_x_min = roi_x_min
        self.roi_x_max = roi_x_max
        self.roi_y_min = roi_y_min
        self.roi_y_max = roi_y_max
        self.roi_z_min = roi_z_min
        self.roi_z_max = roi_z_max
        self.voxel_size = voxel_size
        self.plane_distance_threshold = plane_distance_threshold
        self.plane_ransac_n = plane_ransac_n
        self.plane_num_iterations = plane_num_iterations
        self.cluster_eps = cluster_eps
        self.cluster_min_points = cluster_min_points
        self.human_height_min = human_height_min
        self.human_height_max = human_height_max
        self.human_width_max = human_width_max
        self.human_length_max = human_length_max

    def infer(self, frame: FrameInput) -> List[DetectedObject]:
        points = frame.payload.get("points")
        if points is None or len(points) == 0:
            return []

        # Extract XYZ coordinates
        xyz = points[:, :3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64, copy=False))

        # Crop to ROI
        bbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(self.roi_x_min, self.roi_y_min, self.roi_z_min),
            max_bound=(self.roi_x_max, self.roi_y_max, self.roi_z_max),
        )
        cropped = pcd.crop(bbox)
        if len(cropped.points) == 0:
            return []

        # Downsample
        preprocessed = cropped.voxel_down_sample(voxel_size=self.voxel_size)
        if len(preprocessed.points) < self.plane_ransac_n:
            return []

        # Remove ground plane via RANSAC
        plane_model, inliers = preprocessed.segment_plane(
            distance_threshold=self.plane_distance_threshold,
            ransac_n=self.plane_ransac_n,
            num_iterations=self.plane_num_iterations,
        )
        pcd_no_ground = preprocessed.select_by_index(inliers, invert=True)
        points_no_ground = np.asarray(pcd_no_ground.points, dtype=np.float32)
        if len(points_no_ground) == 0:
            return []

        # Cluster points via DBSCAN
        labels = np.asarray(
            pcd_no_ground.cluster_dbscan(
                eps=self.cluster_eps,
                min_points=self.cluster_min_points,
                print_progress=False,
            ),
            dtype=np.int32,
        )

        detections: List[DetectedObject] = []
        if labels.size == 0:
            return detections

        max_label = int(labels.max())
        if max_label < 0:
            return detections

        for cluster_id in range(max_label + 1):
            cluster_indices = np.where(labels == cluster_id)[0]
            if cluster_indices.size == 0:
                continue
            cluster_pcd = pcd_no_ground.select_by_index(cluster_indices.tolist())
            cluster_bbox = cluster_pcd.get_axis_aligned_bounding_box()
            extent = cluster_bbox.get_extent()

            # Classify bounding box size (human heuristic)
            if not (
                self.human_height_min < extent[2] < self.human_height_max
                and extent[0] < self.human_width_max
                and extent[1] < self.human_length_max
            ):
                continue

            center = cluster_bbox.get_center()
            detections.append(
                DetectedObject(
                    label="Pedestrian",
                    score=1.0,
                    x=float(center[0]),
                    y=float(center[1]),
                    z=float(center[2]),
                    dx=float(extent[0]),
                    dy=float(extent[1]),
                    dz=float(extent[2]),
                    heading=0.0,
                )
            )

        return detections
