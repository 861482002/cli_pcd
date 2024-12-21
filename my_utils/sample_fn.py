# -*- codeing = utf-8 -*-
# @Time : 2024-05-23 17:36
# @Author : 张庭恺
# @File : format.py
# @Software : PyCharm

import numpy as np
from scipy.spatial import KDTree

def random_select_points(pc: np.ndarray, m) -> np.ndarray:
	if m < 0:
		idx = np.arange(pc.shape[0])
		np.random.shuffle(idx)
		return pc[idx, :]
	# pc=np.array(pc)
	# print(len(pc.points))
	# n = len(pc.points)
	n = pc.shape[0]
	print(n)
	replace = False if n >= m else True
	idx = np.random.choice(n, size=(m,), replace=replace)
	return pc[idx, :]


def find_index(full_points: np.ndarray, segment_plane: np.ndarray) -> np.ndarray:
	kd_tree = KDTree(full_points)
	index = kd_tree.query(segment_plane,k = 1)[1]

	return np.array(index)
