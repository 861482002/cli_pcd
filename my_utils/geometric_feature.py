# -*- codeing = utf-8 -*-
# @Time : 2024-07-01 9:56
# @Author : 张庭恺
# @File : geometric_feature.py
# @Software : PyCharm

import numpy as np
import open3d as o3d
from typing import Union

def compute_centroid(points) -> np.ndarray:
    """
    计算点云的质心
    """
    centroid = np.mean(points, axis=0)
    return centroid

def compute_radius(points, centroid):
    """
    计算点云的半径
    """
    distances = np.linalg.norm(points - centroid, axis=1)
    radius = np.max(distances)
    return radius

def compute_height(points):
    """
    计算点云的高度
    """
    min_z = np.min(points[:, 2])
    max_z = np.max(points[:, 2])
    height = max_z - min_z
    return height

# 计算偏心角


# 计算平面相对于xoy面的倾斜角
def compute_inclination(plane: Union[np.ndarray,o3d.geometry.PointCloud]):

    if isinstance(plane, o3d.geometry.PointCloud):
        pass
    else:
        plane = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(plane))

    xoy_norm = np.array([0, 0, 1])
    plane_model, inlier_indices = plane.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    a, b, c, d = plane_model
    plane_norm = np.array([a,b,c])
    cos_angle = (np.dot(plane_norm, xoy_norm)) / (np.linalg.norm(plane_norm) * np.linalg.norm(xoy_norm))
    angle = np.degrees(np.arccos(np.abs(cos_angle)))
    return angle


    pass

#
# if __name__ == "__main__":
#     filename = r"E:\pycharm\20230717\data.txt"  # 替换为你的点云文件路径
#     points = read_point_cloud(filename)
#     centroid = compute_centroid(points)
#     radius = compute_radius(points, centroid)
#     height = compute_height(points, centroid)
#     print("质心:", centroid)
#     print("半径:", radius)
#     print("高度:", height)

