#!/usr/bin/env python
# coding: utf-8
import numpy as np
import pandas as pd
import svgpathtools
from svgpathtools import *
from svgpathtools.path import transform
import matplotlib.pyplot as plt

FIELD_EDGE = 3.6448999

document = svgpathtools.Document('autopathes1.svg')

field_bounds = document.paths()[1].bbox()
path = document.paths()[0]

x1, x2, y1, y2 = field_bounds
path = transform(path, np.array([
	[FIELD_EDGE / (x2 - x1), 0, 0],
	[0, -FIELD_EDGE / (y2 - y1), FIELD_EDGE],
	[0, 0, 1]
]))

# path = path.translated(-path.point(0)) # translate start to 0,0, just for testing
print(path.point(0))


def get_point_along(path, distance):
	if distance <= 0:
		return path.point(0)
	if distance >= path.length():
		return path.point(1)
	return path.point(path.ilength(distance))


def get_curvature_along(path, distance):
	if distance <= 0:
		return 0.
	if distance >= path.length():
		return 0.
	return path.curvature(path.ilength(distance))


LINEAR_WEIGHTS = 1.
ANGULAR_WEIGHT = 0.

FREQUENCY = 10


def generate_dataframe(x, y, motor_weights):
	parameter_dataframe = pd.DataFrame()

	parameter_dataframe['clock_time'] = np.arange(0, len(x)) / FREQUENCY
	parameter_dataframe['x_desired'] = x
	parameter_dataframe['y_desired'] = y
	parameter_dataframe['theta_desired'] = 0

	parameter_dataframe['fl_weight'] = motor_weights
	parameter_dataframe['fr_weight'] = motor_weights
	parameter_dataframe['bl_weight'] = motor_weights
	parameter_dataframe['br_weight'] = motor_weights

	parameter_dataframe['x_weight'] = LINEAR_WEIGHTS
	parameter_dataframe['y_weight'] = LINEAR_WEIGHTS
	parameter_dataframe['theta_weight'] = ANGULAR_WEIGHT

	parameter_dataframe['x_vel_weight'] = 0.
	parameter_dataframe['y_vel_weight'] = 0.
	parameter_dataframe['theta_vel_weight'] = 0.

	parameter_dataframe['x_vel_desired'] = 0.
	parameter_dataframe['y_vel_desired'] = 0.
	parameter_dataframe['angular_vel_desired'] = 0.

	parameter_dataframe.index.name = 'time'

	return parameter_dataframe


step_default = 0.14
path_ending_distance = 0.5
motor_weight_default = 0.01
motor_weight_end = 0.1


def step(curvature):
	return step_default / (0.1 * curvature + 1)


def motor_weights(distance_remaining):
	if distance_remaining < path_ending_distance:
		return motor_weight_end
	else:
		return motor_weight_default


def generate_trajectory(path):
	x = []
	y = []
	motor_weight = []

	path_location = 0
	while path.length() - path_location > 0:
		current_point = get_point_along(path, path_location)
		x.append(np.real(current_point))
		y.append(np.imag(current_point))
		motor_weight.append(motor_weights(path.length() - path_location))

		path_location += step(get_curvature_along(path, path_location))
	return np.array([x, y, motor_weight]).T


trajectory = generate_trajectory(path)
x = trajectory[:, 0]
y = trajectory[:, 1]
weights = trajectory[:, 2]

plt.figure()
plt.plot(x, y, 'b')  # blue
plt.plot(trajectory[:, 0], trajectory[:, 1], 'x c')  # x markers, cyan
plt.plot((trajectory[:, 0])[0:5], (trajectory[:, 1])[0:5], 'x r')  # first 5 points

plt.xlim(0, FIELD_EDGE)
plt.ylim(0, FIELD_EDGE)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

df = generate_dataframe(x, y, weights)

with open('../resources/path.csv', 'w') as file:  # 'x' creates new file, 'w' writes file
	file.write(df.to_csv())





# generate a list of tons of points, instead of only generating a few waypoints
step_discrete = 0.01 # having one point every centimeter should be enough
def generate_points(path):
	distance_along_path = []
	x = []
	y = []

	path_location = 0
	while path.length() - path_location > 0:
		current_point = get_point_along(path, path_location)

		distance_along_path.append(path_location)
		x.append(np.real(current_point))
		y.append(np.imag(current_point))

		path_location += step_discrete
	return np.array([distance_along_path, x, y]).T


def generate_dataframe_points(distance_along_path, x, y):
	parameter_dataframe = pd.DataFrame()

	parameter_dataframe['distance'] = distance_along_path
	parameter_dataframe['x_desired'] = x
	parameter_dataframe['y_desired'] = y

	parameter_dataframe.index.name = 'point'

	return parameter_dataframe

points_list = generate_points(path)
distance_along_path = points_list[:, 0]
x = points_list[:, 1]
y = points_list[:, 2]

df = generate_dataframe_points(distance_along_path, x, y)

# find the second derivative in each direction with respect to distance numerically
second_derivative_x = np.gradient(np.gradient(x, distance_along_path), distance_along_path)
second_derivative_y = np.gradient(np.gradient(y, distance_along_path), distance_along_path)

# find the curvature
curvature = np.sqrt(second_derivative_x ** 2 + second_derivative_y ** 2)
curvature = np.convolve(curvature, np.ones(10) / 10, mode='same')
# plot, with the curvature as the color
plt.figure()

curvature_plot = curvature
plt.scatter(x, y, c=curvature_plot, cmap='gray')
plt.xlim(0, FIELD_EDGE)
plt.ylim(0, FIELD_EDGE)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

# print the curvature with respect to distance
plt.figure()
# plt.plot(distance_along_path, curvature)
plt.plot(distance_along_path, curvature_plot)
plt.xlabel('distance along path')
plt.ylabel('curvature')
plt.show()



with open('../resources/blue_triangle_auto/test.csv', 'w') as file:  # 'x' creates new file, 'w' writes file
	file.write(df.to_csv())
	