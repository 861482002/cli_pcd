# -*- coding: utf-8 -*-
"""
@file: double_plane_ralationship.py
@time: 2024/11/18 下午7:19
@author: ztk
"""
import numpy as np
import open3d as o3d
from typing import Union

from .geometric_feature import compute_centroid

PointCloud = o3d.geometry.PointCloud
Vector3dVector = o3d.utility.Vector3dVector
# 计算两个点云平面的偏心误差
ndarray = np.ndarray


def get_plane(pcd: ndarray):
	vector = o3d.utility.Vector3dVector(pcd)
	pcd = PointCloud(vector)
	plane, _ = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
	return plane


# 将A平面的质心投影到B平面上
def projection_A2B(pcd1: ndarray, pcd2: ndarray):
	center1 = compute_centroid(pcd1)

	plane1 = get_plane(pcd1)
	plane2 = get_plane(pcd2)

	A1, B1, C1, D1 = plane1
	A2, B2, C2, D2 = plane2

	# 将center1投影到plane2上
	distance_point_plane = ((center1[0] * A2 + center1[1] * B2 + center1[2] * C2 + D2) /
	                        (np.sqrt(A2 ** 2 + B2 ** 2 + C2 ** 2)))

	project_x = center1[0] - distance_point_plane * (A2 / np.sqrt(A2 ** 2 + B2 ** 2 + C2 ** 2))
	project_y = center1[1] - distance_point_plane * (B2 / np.sqrt(A2 ** 2 + B2 ** 2 + C2 ** 2))
	project_z = center1[2] - distance_point_plane * (C2 / np.sqrt(A2 ** 2 + B2 ** 2 + C2 ** 2))
	projection_center = np.array([project_x, project_y, project_z])

	return projection_center


# 计算两个平面的偏心度
def error_eccentricity(pcd1: ndarray, pcd2: ndarray):
	center1 = compute_centroid(pcd1)
	center2 = compute_centroid(pcd2)

	# 将center1投影到plane2上
	# 其实函数里面会计算center1，上面第一行的操作多余
	new_center1 = projection_A2B(pcd1, pcd2)

	distance = np.linalg.norm(new_center1 - center2)

	return distance


# 计算平行度
def parallelism(pcd1, pcd2):
	plane1 = get_plane(pcd1)
	plane2 = get_plane(pcd2)

	A1, B1, C1, D1 = plane1
	A2, B2, C2, D2 = plane2

	n1 = np.array([A1, B1, C1])
	n2 = np.array([A2, B2, C2])

	dot_product = np.dot(n1, n2).astype(np.float32)
	n1_norm = np.linalg.norm(n1).astype(np.float32)
	n2_norm = np.linalg.norm(n2).astype(np.float32)

	arccos = np.arccos(dot_product / (n1_norm * n2_norm))

	cos_theta = np.clip(arccos, -1, 1)
	cos_theta = np.nan_to_num(cos_theta, nan=0)
	angle = np.degrees(cos_theta)
	return angle


# 计算两个平面的垂直度
def perpendicularity(pcd1, pcd2):
	return


def fit_cylinder_axis(points):
	"""
	使用最小二乘法拟合圆柱点云的轴线
	参数:
	points (numpy.ndarray): 点云数据，形状为 (N, 3)，其中 N 是点的数量
	返回:
	tuple: (center, axis_direction) 其中 center 是质心，axis_direction 是轴线方向向量
	"""
	# 计算质心
	center = np.mean(points, axis=0)

	# 将点云数据平移到原点
	centered_points = points - center

	# 构建协方差矩阵
	cov_matrix = np.cov(centered_points, rowvar=False)

	# 特征值分解
	eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

	# 找到最大特征值对应的特征向量，这就是轴线方向
	axis_direction = eigenvectors[:, np.argmax(eigenvalues)]
	print("axis_direction", axis_direction)
	return center, axis_direction


def project_points_to_axis(points, axis_direction):
	"""
	将点云数据投影到轴线上
	参数:
	points (numpy.ndarray): 点云数据，形状为 (N, 3)，其中 N 是点的数量
	axis_direction (numpy.ndarray): 轴线方向向量，形状为 (3,)
	返回:
	numpy.ndarray: 投影后的坐标值，形状为 (N,)
	"""
	# 计算轴线方向向量的模
	axis_norm = np.linalg.norm(axis_direction)

	# 归一化轴线方向向量
	axis_direction /= axis_norm

	# 计算点云数据在轴线方向上的投影
	projections = np.dot(points, axis_direction)

	return projections


def calculate_cylinder_height(points):
	"""
	计算圆柱点云的高度
	参数:
	points (numpy.ndarray): 点云数据，形状为 (N, 3)，其中 N 是点的数量
	返回:
	float: 圆柱的高度
	"""
	# 拟合圆柱轴线
	center, axis_direction = fit_cylinder_axis(points)

	# 将点云数据投影到轴线上
	projections = project_points_to_axis(points, axis_direction)

	# 计算投影的最大值和最小值
	max_projection = np.max(projections)
	min_projection = np.min(projections)

	# 计算高度
	height = max_projection - min_projection

	return height

	pass
#TODO 计算偏心角
def compute_angle_eccentricity(plane1: Union[np.ndarray,o3d.geometry.PointCloud], plane2: Union[np.ndarray,o3d.geometry.PointCloud]):

	centroid1 = compute_centroid(plane1)
	centroid2 = compute_centroid(plane2)
	# 计算两个质心的连线

	direction_ab = centroid2 - centroid1
	x_direction = np.array([1, 0, 0])
	cos_angle = np.dot(direction_ab, x_direction) / (np.linalg.norm(direction_ab) * np.linalg.norm(x_direction))
	angle = np.degrees(np.arccos(np.abs(cos_angle))).astype(np.float16)
	return angle



	pass
