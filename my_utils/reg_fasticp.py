# -*- coding: utf-8 -*-
"""
@file: reg_fasticp.py
@time: 2024/11/6 下午3:31
@author: ztk

"""
import open3d as o3d
import numpy as np

def fast_icp_registration(source, target, voxel_size = 0.05, max_iteration=1000):
    # fast-icp 算法
    # 读取源点云和目标点云
    source = o3d.io.read_point_cloud("source.pcd")  # 替换为源点云文件路径
    target = o3d.io.read_point_cloud("target.pcd")  # 替换为目标点云文件路径

    # 初步可视化未对齐的点云
    o3d.visualization.draw_geometries([source, target], window_name="Initial Point Clouds")

    # 下采样以减少数据量
    # voxel_size = 0.05  # 下采样体素大小，可以根据需求调整
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    # 计算法向量以便使用点到平面的 ICP 配准
    source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

    # 定义配准距离阈值
    threshold = 0.1  # 调整此值来控制匹配精度

    # 使用点到平面的误差进行 ICP 配准（更快收敛）
    icp_result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, threshold, np.eye(4),  # 初始变换矩阵为单位矩阵
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

# 输出配准结果
# print("Fast ICP 配准完成")
# print("估计的变换矩阵：\n", icp_result.transformation)
# print("匹配的 Fitness 值:", icp_result.fitness)
# print("匹配的 RMSE 值:", icp_result.inlier_rmse)

# 应用最终变换到原始分辨率的源点云
# source.transform(icp_result.transformation)
#
# # 可视化最终配准结果
# o3d.visualization.draw_geometries([source, target], window_name="Fast ICP Registration Result", width=800, height=600)
