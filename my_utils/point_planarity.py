# -*- codeing = utf-8 -*-
# @Time : 2024-05-13 15:06
# @Author : 张庭恺
# @File : point_planarity.py
# @Software : PyCharm

import numpy as np

def fit_plane(points):
	"""
	使用最小二乘法拟合点云中的平面。

	参数:
	points (numpy.ndarray): 点云数据，形状为(N, 3)，其中N为点的数量。

	返回:
	numpy.ndarray: 拟合平面的系数[A, B, C, D]。
	"""
	# 确保数据是Numpy数组
	points = np.array(points)

	# 计算中心点
	centroid = np.mean(points, axis=0)

	# 中心化数据减去中心点
	centered_points = points - centroid

	# 构造矩阵并求解
	M = np.dot(centered_points.T, centered_points)
	[U, S, V] = np.linalg.svd(M)
	normal = V[-1]
	D = -np.dot(normal, centroid)

	return np.concatenate((normal, [D]))


def calculate_planarity(points, plane):
	"""
	计算点云相对于已知平面的平面度（这里以所有点到平面距离的标准差作为度量）。

	参数:
	points (numpy.ndarray): 点云数据。
	plane (numpy.ndarray): 平面的系数[A, B, C, D]。

	返回:
	float: 点云的平面度（标准差）。
	"""
	distances = np.abs(np.dot(points, plane[:3]) + plane[3])
	return np.std(distances)


# 示例点云数据
# points = np.array([[1, 2, 3], [2, 3, 4], [3, 5, 6], [4, 6, 7], [5, 7, 8]])
# # cloud = o3d.geometry.PointCloud()
# # cloud.points = o3d.utility.Vector3dVector(points)
# # o3d.visualization.draw_geometries([cloud])
# # 拟合平面
# plane_coefficients = fit_plane(points)
#
# # 计算平面度
# planarity = calculate_planarity(points, plane_coefficients)
# print(f"拟合平面的系数为: {plane_coefficients}")
# print(f"点云的平面度（标准差）为: {planarity}")