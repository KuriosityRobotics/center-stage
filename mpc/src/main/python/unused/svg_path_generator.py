import numpy as np
import pandas as pd
import svgpathtools
from svgpathtools import *
from svgpathtools.path import transform
import matplotlib.pyplot as plt

FIELD_EDGE = 3.6448999 # metres

document = svgpathtools.Document('auto_pathes.svg')
csv_destination = '../resources/red_triangle_auto/red_cycle_to_medium.csv'
file_generator_keyword = 'x' # 'x' creates new file, 'w' writes file

field_bounds = document.paths()[-1].bbox() # the path that is scaled to the size of the field (always the last path)
path = document.paths()[3] # the path you want to generate (starting from index 0)
# the most recent path added will be index 0, and everything will get moved over by one

x1, x2, y1, y2 = field_bounds
path = transform(path, np.array([
	[FIELD_EDGE / (x2 - x1), 0, 0],
	[0, -FIELD_EDGE / (y2 - y1), FIELD_EDGE],
	[0, 0, 1]
]))

def get_point_along(path, distance):
	if distance <= 0:
		return path.point(0)
	if distance >= path.length():
		return path.point(1)
	return path.point(path.ilength(distance))

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
plt.title(csv_destination)
plt.show()

# print the curvature with respect to distance
plt.figure()
# plt.plot(distance_along_path, curvature)
plt.plot(distance_along_path, curvature_plot)
plt.xlabel('distance along path')
plt.ylabel('curvature')
plt.title(csv_destination)
plt.show()



with open(csv_destination, file_generator_keyword) as file:  # 'x' creates new file, 'w' writes file
	file.write(df.to_csv())