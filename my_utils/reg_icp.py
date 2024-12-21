# -*- coding: utf-8 -*-
"""
@file: reg_icp.py
@time: 2024/10/31 下午3:03
@author: ztk

"""
import open3d as o3d
import numpy as np

def icp_registration(source, target, threshold = 0.02, init_source_to_target_transform = np.eye(4)):
    # 原始icp 算法
    # 读取源点云和目标点云
    source = o3d.io.read_point_cloud(source)  # 替换为源点云文件路径
    target = o3d.io.read_point_cloud(target)  # 替换为目标点云文件路径

    # 初步查看源点云和目标点云
    print("源点云:", source)
    print("目标点云:", target)

    # 可视化初始状态
    o3d.visualization.draw_geometries([source, target], window_name="Initial Point Clouds")

    # 定义配准的距离阈值
    # threshold = 0.02  # 调整此值以控制匹配的精度和速度

    # 执行原始 ICP 算法，使用点到点的误差
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init_source_to_target_transform,  # 初始变换矩阵为单位矩阵
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

# 输出结果信息
# print("ICP 配准完成")
# print("估计的变换矩阵：\n", icp_result.transformation)
# print("匹配的 Fitness 值:", icp_result.fitness)
# print("匹配的 RMSE 值:", icp_result.inlier_rmse)

# # 将源点云应用 ICP 变换
# source.transform(icp_result.transformation)
#
# # 可视化配准结果
# o3d.visualization.draw_geometries([source, target], window_name="ICP Registration Result", width=800, height=600)
