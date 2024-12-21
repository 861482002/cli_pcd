# -*- codeing = utf-8 -*-
# @Time : 2024-05-22 11:15
# @Author : 张庭恺
# @File : arguments.py
# @Software : PyCharm

from dataclasses import dataclass
from argparse import ArgumentParser


def get_args():
	parser = ArgumentParser()
	parser.add_argument('--src', type=str)
	parser.add_argument('--tar', type=str)
	parser.add_argument('--plane' , type = int,default=12)      #展示 0 ，11 两个面的效果
	args = parser.parse_args()
	return args


@dataclass()
class Config:
	'''
	1.配准模型的参数

	2.pointnet网络参数

	3.CAD离散成点云的参数
	'''
	# 1、配准模型参数
	use_reg: bool = False
	reg_model_path: str = r'D:\A_point-cloud-proj\PCReg\pth\test_min_loss.pth'
	in_dim: int = 3
	niters: int = 8
	gn: bool = False
	device:str = 'cuda'
	infer_npts:int = 30000
	voxelsize: float = 5


	# 2、pointnet网络参数
	use_pointnet: bool = False
	num_class:int = 40      #预训练模型设置的参数
	normal_channel:bool = False
	pointnet_model_path:str = r'D:\A_point-cloud-proj\Pointnet_Pointnet2\log\classification\pointnet2_cls_ssg\checkpoints\best_model.pth'

	# CAD离散成点云的参数
	sample_num:int = 90000

	# faiss参数
	topk:int = 1

	# 选取平面的参数
	plane:str = 'curve'
	pass


if __name__ == '__main__':
	cfg = Config()
	# test_pointnet(cfg)
	cfg.text = 'nihao'
	pass
