# -*- coding: utf-8 -*-
"""
@file: cli_integration.py
@time: 2024/12/3 下午4:49
@author: ztk

"""
import os

import numpy as np

# import numpy as np

# -*- codeing = utf-8 -*-
# @Time : 2024-05-22 17:15
# @Author : 张庭恺
# @File : integration.py
# @Software : PyCharm

from arguments import Config, get_args
from my_utils import *


import json

PointCloud = o3d.geometry.PointCloud
ndarray = np.ndarray
def parse_strv1(s:str):
	# '5.,55.77,77.87;5.,88.37,126.72;5.,79.48,138.69'
	s_lst = s.split(';')

	all_points = []
	for s_ in s_lst:
		xyz_lst = s_.split(',')
		xyz = [float(xyz_) for xyz_ in xyz_lst]
		all_points.append(xyz)
	edge = np.array(all_points)
	return edge
def parse_strv2(s:str):
	'''
	Parameters
	s:
	-69.1103 34.7657 259.138
	-69.1103 34.7657 259.138
	-69.1103 34.7657 259.138
	-69.1103 34.7657 259.138
	-69.1103 34.7657 259.138
	-69.1103 34.7657 259.138
	-69.1103 34.7657 259.138
	-69.1103 34.7657 259.138
	'''

	pass

def bindPlane(src_files:str, target_file:str,args):

	args = args
	cfg = Config()
	integration = Integration(cfg)

	# TODO 第一个参数    实测列表-每个元素对应一个配准、分割后的点云文件名(url)
	# 1、分割后的所有点云文件
	postreg_source_files = src_files  # type: str
	postreg_source_files = postreg_source_files.split('#')

	# postreg_source : list = field(default=[],metadata = {'help':'300个分割后的实测点云表面'})
	# target : list = field(default=[],metadata = {'help':'3个分割后的标准点云表面'})
	postreg_source = []
	for file_name in postreg_source_files:
		cur_pcd = read_txt2(file_name)
		# 滤波
		# cur_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cur_pcd))
		# cur_pcd,_ = cur_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2)
		# cur_pcd = np.asarray(cur_pcd.points)
		postreg_source.append(cur_pcd)

	# TODO 第二个参数 选取的两个配合面的边界值
	# 2、要选取的目标文件
	target_file = target_file  # type: str
	target_pcd = read_txt2(target_file)



	# 3、计算传过来选定的两个配合面各自的特征
	# TODO 添加半径特征
	selected_center1 = compute_centroid(target_pcd)[None, :]  # shape[1,3]
	selected_radius1 = compute_radius(target_pcd, selected_center1) # shape[1]
	selected_radius1 = np.array([selected_radius1])[None, :]
	selected_feature1 = np.concatenate([selected_center1, selected_radius1], axis=1)  # shape[1,4]
	# 再加一个平面度的判断条件
	tar_plane = fit_plane(target_pcd)
	tar_planarity = calculate_planarity(target_pcd, tar_plane)
	tar_planarity = np.array([tar_planarity])[None, :]
	selected_feature1 = np.concatenate([selected_feature1, tar_planarity], axis=1)
	# 无量纲化
	# mean,std = selected_feature1.mean(axis=1), selected_feature1.std(axis=1)
	# selected_feature1 = (selected_feature1 - mean) / std

	# 4、计算所有配准后分割出来的实测点云表面的特征
	centers_ply = integration.comput_center(postreg_source)  # shape[n,3]
	centers_ply_lst = np.split(centers_ply, centers_ply.shape[0], axis=0)  # list[3]
	radius_ply_lst = []
	planarity_ply_lst = []
	for pcd, center in zip(postreg_source, centers_ply_lst):
		radius = compute_radius(pcd, center)
		radius_ply_lst.append(radius)
		# 平面度
		plane = fit_plane(pcd)
		planarity_ply_lst.append(calculate_planarity(pcd, plane))

	radius_ply_lst = np.array(radius_ply_lst)[:, None]  # shape[n,1]
	all_feature = np.concatenate([centers_ply, radius_ply_lst], axis=1)         # shape[n,4]
	# 添加平面度
	planarity_ply_lst = np.array(planarity_ply_lst)[:, None]  # shape[n,1]
	all_feature = np.concatenate([all_feature, planarity_ply_lst], axis=1)      # shape[n,5]

	# 无量纲化
	# mean,std = all_feature.mean(axis=0), all_feature.std(axis=0)
	# all_feature = (all_feature - mean) / std

	print('_' * 20, '特征提取完成\t\t\t', '_' * 20)


	# 匹配到的文件路径
	# TODO 这里实现多线程或者多进程处理提高并行计算效率，使用任务分组计算

	# 5、匹配第一个配合面对应的点云
	matched1_files = feature_match_cos_multi(all_feature, postreg_source_files, selected_feature1, topk=1)
	# matched1_files = feature_match_cos_multi(centers_ply, postreg_source_files, selected_center1, topk=1)
	matched1_idxs = [postreg_source_files.index(file) for file in matched1_files]

	# 6、匹配第二个配合面对应的点云
	# matched2_files = feature_match_cos_multi(all_feature, postreg_source_files, selected_feature2, topk=1)
	# matched2_idxs = [postreg_source_files.index(file) for file in matched2_files]


	print('_' * 20, '匹配完成\t\t\t', '_' * 20)

	print('匹配到的第一个配合面：' , matched1_files)
	# print('匹配到的第二个配合面：' , matched2_files)

	selected_cad_plane_show = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_pcd))
	selected_cad_plane_show.paint_uniform_color([0, 1, 0])
	# 7、计算第一个配合面的特征值
	res_dict = {}
	for matched_idx in matched1_idxs:

		matched_filename = postreg_source_files[matched_idx]
		print(postreg_source_files[matched_idx])
		match_pcd = postreg_source[matched_idx]
		match_cad_show = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(match_pcd))
		match_cad_show.paint_uniform_color([1, 0, 0])
		# show_files.append(match_cad_show)
		o3d.visualization.draw_geometries([selected_cad_plane_show, match_cad_show])

		plane = fit_plane(match_pcd)
		matched_planarity = calculate_planarity(match_pcd, plane)

		matched_centroid = compute_centroid(match_pcd)
		matched_radius = compute_radius(match_pcd, matched_centroid)
		matched_height = compute_height(match_pcd)

		res_dict[matched_filename] = {
			'planarity': str(matched_planarity),
			'centroid': str(matched_centroid),
			'radius': str(matched_radius),
		}
		print('matched_planarity:', matched_planarity)
		print('matched_centroid:', matched_centroid)

		print('matched_radius:', matched_radius)
		# print('matched_height:', matched_height)


	json_res = json.dumps(res_dict)
	return json_res


class Integration:
	'''
	集成所有部分的类
	需要做的事情
	1.构建配准模型
	2.构建pointnet++模型
	3.包含CAD离散方法
	4.包含特征匹配方法
	5.分割方法
	'''

	segmented_ply: List = []
	segmented_cad: List = []
	postseg_dir: str = None

	def __init__(self, cfg: Config):
		self.cfg = cfg
		# self.reg_model = Registration(cfg) if cfg.use_reg else None
		# self.pointnet_model = PointNet(cfg) if cfg.use_pointnet else None
		self.postreg_ply = None
		pass


	def reg_ply_cad_icp(self, source: PointCloud, target: PointCloud, voxelsize: float = 5) -> Tuple[
		PointCloud, ndarray]:
		'''
		使用ICP方法配准两个模型
		Parameters
		----------
		source  :待配准的点云
		target  :目标点云
		voxelsize   :下采样的时候体素的大小

		Returns
		-------

		'''
		# 首先进行下采样、法向量估计、快速点特征直方图(FPFH)
		ply_down, ply_fpfh = preprocess_point_cloud(source, voxelsize)
		cad_down, cad_fpfh = preprocess_point_cloud(target, voxelsize)

		# 进行ICP配准
		# 使用的方法是RANSAC 随机一致性采样法
		result_ransac = execute_global_registration(ply_down, cad_down, ply_fpfh, cad_fpfh, voxel_size=voxelsize)

		# 精配准
		result_icp = refine_registration(ply_down, cad_down, voxelsize, result_ransac)

		# 配准后的点云
		# 这个方法是在原地修改
		post_reg_ply = copy.deepcopy(source)
		post_reg_ply = post_reg_ply.transform(result_icp.transformation)

		return post_reg_ply, result_icp.transformation

	def transformation(self, point_cloud: PointCloud, transformation: ndarray) -> PointCloud:

		point_cloud_copy = copy.deepcopy(point_cloud)
		point_cloud_copy.transform(transformation)
		return point_cloud_copy

	def comput_center(self, points_list: List[np.ndarray]) -> np.ndarray:
		'''
		计算点云的质心
		Parameters
		----------
		points_list:List [ndarray:[n_points , 3]]

		Returns
		-------
		ndarray:[n,3]
		'''
		centroid_list = []
		for cur_points in points_list:
			centroid = compute_centroid(cur_points)
			centroid_list.append(centroid)

		centroid_list = np.vstack(centroid_list)
		return centroid_list


	def segment_postreg_ply(self, ply_file: Optional[str] = None, out_dir: Optional[str] = None) -> List:
		# TODO 先写死，后续调用c++的程序进行分割
		segmented_ply_dir = './data/segmentply'
		segmented_ply_dir_list = os.listdir(segmented_ply_dir)
		self.postseg_ply_dir = out_dir
		for ply in segmented_ply_dir_list:
			cur_ply_path = os.path.join(segmented_ply_dir, ply)
			cur_pcd = read_ply(cur_ply_path)
			points = np.asarray(cur_pcd.points)
			self.segmented_ply.append(points)
		# shape不一样
		return self.segmented_ply

	def pc_normalize(self, pc: ndarray) -> ndarray:
		centroid = np.mean(pc, axis=0)
		pc = pc - centroid
		m = np.max(np.sqrt(np.sum(pc ** 2, axis=1)))
		pc = pc / m
		return pc

	def segment_cad(self, cad_file: Optional[str] = None, out_dir: Optional[str] = None) -> List:
		# TODO 先写死，后续调用c++的程序进行分割

		segmented_cad_dir = './data/segmentply'
		segmented_cad_dir_list = os.listdir(segmented_cad_dir)
		self.postseg_cad_dir = out_dir
		for cad in segmented_cad_dir_list:
			cur_cad_path = os.path.join(segmented_cad_dir, cad)
			cur_pcd = read_ply(cur_cad_path)
			points = np.asarray(cur_pcd.points)
			self.segmented_cad.append(points)
		# shape不一样
		return self.segmented_cad
		pass

	def select_plane(self, plane_list: List[ndarray], idx) -> np.ndarray:
		# TODO 先写死，后续会添加参数作为判断条件
		'''


		Parameters
		----------
		plane_list
		idx


		边界表示：str 可能多个面，不同面之间用分号表示
		Returns
		-------

		'''
		idx = idx
		pcd = plane_list[idx]
		return pcd

		pass

	def faiss_match(self, query: np.ndarray, query_dir: str, Index: np.ndarray):
		# TODO 特征检索匹配
		matched_file = feature_match_single(query, query_dir, Index)
		self.matched_file = matched_file
		pass

	def visualize(self, pcd1: ndarray, pcd2: ndarray, *args):
		# TODO 可视化
		visualize_point_clouds(pcd1, pcd2, *args)

		pass


if __name__ == '__main__':
	# 初始化
	args = get_args()
	src_files = '#'.join([ os.path.join('./part1',file) for file in os.listdir('./part1') if len(file) == 7])
	# tar_file = './part1/PG5.txt'
	tar_file = './数据/目标分割面/PG2.txt'
	bind_plane = bindPlane(src_files,tar_file,args)
	# bind_plane = bindPlane(args.src,args.tar,args)
	print(bind_plane)
	# path1 = './part1/PG0.txt'
	# path2 = './part1/PG1.txt'
	# path3 = './part1/PG2.txt'
	# path4 = './part1/PG3.txt'
	# all_path = [path1,path2,path3,path4]
	# all_path = '#'.join(all_path)
	# # './part1/PG0.txt#./part1/PG1.txt#./part1/PG2.txt#./part1/PG3.txt#./part1/PG4.txt#./part1/PG5.txt'
	# print('nini')

	# --src ./part1/PG0.txt#./part1/PG1.txt#./part1/PG2.txt#./part1/PG3.txt#./part1/PG4.txt#./part1/PG5.txt --tar ./数据/目标分割面/PG6.txt
	pass
