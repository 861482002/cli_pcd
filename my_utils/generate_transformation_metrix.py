# -*- coding: utf-8 -*-
"""
@file: generate_transformation_metrix.py
@time: 2024/11/7 下午3:13
@author: ztk

"""

import math
import numpy as np


def generate_rotation_x_matrix(theta):
	mat = np.eye(3, dtype=np.float32)
	mat[1, 1] = math.cos(theta)
	mat[1, 2] = -math.sin(theta)
	mat[2, 1] = math.sin(theta)
	mat[2, 2] = math.cos(theta)
	return mat

def generate_rotation_y_matrix(theta):
	mat = np.eye(3, dtype=np.float32)
	mat[0, 0] = math.cos(theta)
	mat[0, 2] = math.sin(theta)
	mat[2, 0] = -math.sin(theta)
	mat[2, 2] = math.cos(theta)
	return mat

def generate_rotation_z_matrix(theta):
	mat = np.eye(3, dtype=np.float32)
	mat[0, 0] = math.cos(theta)
	mat[0, 1] = -math.sin(theta)
	mat[1, 0] = math.sin(theta)
	mat[1, 1] = math.cos(theta)
	return mat

def generate_random_rotation_matrix(angle1=-45, angle2=45):
	thetax, thetay, thetaz = np.random.uniform(angle1, angle2, size=(3,))
	matx = generate_rotation_x_matrix(thetax / 180 * math.pi)
	maty = generate_rotation_y_matrix(thetay / 180 * math.pi)
	matz = generate_rotation_z_matrix(thetaz / 180 * math.pi)
	return np.dot(matz, np.dot(maty, matx))


def generate_random_tranlation_vector(range1=-1, range2=1):
	tranlation_vector = np.random.uniform(range1, range2, size=(3,)).astype(np.float32)
	return tranlation_vector


def generate_certain_rotation_matrix(angle_x, angle_y, angle_z) -> np.ndarray:
	'''


	Parameters
	----------
	angle_x
	angle_y
	angle_z

	Returns
	-------
    返回ndarray:[3,3]
	'''
	thetax, thetay, thetaz = angle_x / 180 * math.pi, angle_y / 180 * math.pi, angle_z / 180 * math.pi
	matx = generate_rotation_x_matrix(thetax)
	maty = generate_rotation_y_matrix(thetay)
	matz = generate_rotation_z_matrix(thetaz)
	return np.dot(matz, np.dot(maty, matx))


def generate_certain_tranlation_vector(x, y, z) -> np.ndarray:
	'''

	Parameters
	----------
	x
	y
	z

	Returns
	-------
    返回ndarray:[3]
	'''
	tranlation_vector = np.array([x, y, z]).astype(np.float32)
	return tranlation_vector


def transform(pc, R, t=None):
	pc = np.dot(pc, R.T)
	if t is not None:
		pc = pc + t
	return pc
