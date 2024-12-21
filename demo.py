# -*- coding: utf-8 -*-
"""
@file: demo.py
@time: 2024/12/9 下午2:00
@author: ztk

"""

from my_utils import *
import open3d as o3d
import numpy as np


def show():
	src_files = [os.path.join('./零件2数据/目标', file) for file in os.listdir('./零件2数据/目标') if len(file) == 7]
	colors = [np.random.rand(3) for i in range(len(src_files))]
	show_3d_files(src_files,colors)


def parse_txt_xyzrgb(filename):

	pcd_xyzrgb = np.loadtxt(filename, delimiter=' ')
	points = pcd_xyzrgb[:, :3]
	colors = pcd_xyzrgb[:, 3:]
	pcd = PointCloud(o3d.utility.Vector3dVector(points))
	return pcd


if __name__ == '__main__':
	# pcd_ndarray = read_txt2('./零件1数据/目标/PG6.txt')             # 目标的点云 平面度是 0.008 质心  [1.49,-9.2,230]   空心圆
	# pcd_ndarray = read_txt2('./零件1数据/目标/PG2.txt')             # 目标的点云 平面度是 10 质心  [1.5,-9.3,280]   上半部分的杯子




	# pcd_ndarray = read_txt2('./零件1数据/实测/PG0.txt')             # 实测点云0   平面度是 10 质心  [1.6，-9.1，280] 上半部分的杯子
	# pcd_ndarray = read_txt2('./零件1数据/实测/PG1.txt')             # 实测点云1   平面度是 0.14 质心  [1.75，-9.8，239] 空心圆
	# pcd_ndarray = read_txt2('./零件1数据/实测/PG2.txt')             # 实测点云2   平面度是 1.4 质心  [1.3，-8.9，234.4] 竖着的柱面
	# pcd_ndarray = read_txt2('./零件1数据/实测/PG3.txt')             # 实测点云3   平面度是 0.08 质心  [2.0，-9.0，239.9] 圆面
	# pcd_ndarray = read_txt2('./零件1数据/实测/PG4.txt')             # 实测点云4   平面度是 0.13 质心  [1.8，-8.6，230] 空心圆
	# pcd_ndarray = read_txt2('./零件1数据/实测/PG5.txt')             # 实测点云5   平面度是 1.31 质心  [1.8，-8.7，235] 竖着的柱面


	# pcd_ndarray = read_txt2('./零件3数据/目标/PG4.txt')             # 目标的点云 平面度是 0.72 质心  [1.8,-8.4,147]   竖着的柱面
	# pcd_ndarray = read_txt2('./零件3数据/目标/PG5.txt')             # 目标的点云 平面度是 0.01 质心  [1.4,-8.2,149.9]   空心圆

	# pcd_ndarray = read_txt2('./零件3数据/实测/PG0.txt')             # 实测点云0   平面度是 0.17 质心  [1.67，-9.2，55] 小圆面
	# pcd_ndarray = read_txt2('./零件3数据/实测/PG1.txt')             # 实测点云1   平面度是 14 质心  [1.37，-8.9，100] 外侧大圆柱
	# pcd_ndarray = read_txt2('./零件3数据/实测/PG2.txt')             # 实测点云2   平面度是 11 质心  [1.67，-8.9，102] 内侧小圆柱
	# pcd_ndarray = read_txt2('./零件3数据/实测/PG3.txt')             # 实测点云3   平面度是 0.21 质心  [1.7，-9.0，50] 大圆面
	# pcd_ndarray = read_txt2('./零件3数据/实测/PG4.txt')             # 实测点云4   平面度是 0.11 质心  [2.1，-8.6，149] 空心圆
	# pcd_ndarray = read_txt2('./零件3数据/实测/PG5.txt')             # 实测点云5   平面度是 0.67 质心  [2.1，-7.9，147] 竖着的柱面
	#
	#
	# plane = fit_plane(pcd_ndarray)
	# planarity = calculate_planarity(pcd_ndarray, plane)
	# print(f'平面度：{planarity}',type(planarity))
	# centroid = compute_centroid(pcd_ndarray)
	# print(f'质心：{centroid}')
	# pcd_point = o3d.utility.Vector3dVector(pcd_ndarray)
	# pcd = PointCloud(pcd_point)
	# o3d.visualization.draw_geometries([pcd,axis])

	pcd1 = parse_txt_xyzrgb('点云数据/零件A1 - tesselated.txt')
	pcd2 = parse_txt_xyzrgb('点云数据/零件A2 - tesselated.txt')
	# pcd2 = o3d.io.read_point_cloud('1.ply')
	o3d.visualization.draw_geometries([pcd2, axis])

	pass
