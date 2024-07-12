import glob
from multiprocessing import Pool
from multiprocessing.pool import AsyncResult

import numpy as np
import matplotlib.pyplot as plt
from numba import njit

from drive_simulation import DriveParameters, RobotState, RobotCommand, R, _acceleration, _net_torque
from mecanum_data import DataSeries
from mecanum_data import T

def simulate(sample: DataSeries, robot: DriveParameters, graph_velocity=False, graph_position=False, graph_motors=False):
	robot_position, robot_torque, simulated_velocity = _simulate(
		sample.x_position, sample.y_position, sample.angle,
		sample.x_velocity, sample.y_velocity, sample.angular_velocity,
		sample.battery_voltage,
		sample.fl, sample.fr, sample.bl, sample.br,
		robot.motor_constant_e, robot.motor_constant_t, robot.armature_resistance,
		robot.dynamic_friction, robot.directional_friction, robot.M_r, robot.M_w
	)

	if graph_velocity:
		plt.figure()
		plt.plot(sample.time, sample.x_velocity, label='X Velocity (Measured)')
		plt.plot(sample.time, sample.y_velocity, label='Y Velocity (Measured)')
		plt.plot(sample.time, sample.angular_velocity, label='Angular Velocity (Measured)')

		plt.plot(sample.time, simulated_velocity[:, 0], label='X Velocity (Simulated)')
		plt.plot(sample.time, simulated_velocity[:, 1], label='Y Velocity (Simulated)')
		plt.plot(sample.time, simulated_velocity[:, 2], label='Angular Velocity (Simulated)')
		# plt.plot(sample.time, sample.angle, label='Angle (Measured)')
		# plt.plot(sample.time, robot_position[:, 2], label='Angle (Simulated)')

		plt.title(f"velocity {sample.name}")
		plt.legend()
		plt.show()
		import os
		if (os.path.isdir('/var/www/max/mpc_simulation')):
			plt.savefig(f"/var/www/max/mpc_simulation/velocity_{sample.name.split('/')[-1]}.png")
			plt.close()

	if graph_position:
		plt.figure()
		plt.plot(sample.time, sample.angle, label='Angle (Measured)')
		plt.plot(sample.time, sample.x_position, label='X Position (Measured)')
		plt.plot(sample.time, sample.y_position, label='Y Position (Measured)')

		plt.plot(sample.time, robot_position[:, 2], label='Angle (Simulated)')
		plt.plot(sample.time, robot_position[:, 0], label='X Position (Simulated)')
		plt.plot(sample.time, robot_position[:, 1], label='Y Position (Simulated)')

		plt.title(f"position {sample.name}")
		plt.legend()
		plt.show()
		import os
		if (os.path.isdir('/var/www/max/mpc_simulation')):
			plt.savefig(f"/var/www/max/mpc_simulation/position_{sample.name.split('/')[-1]}.png")
			plt.close()

	# torque
	if graph_motors:
		plt.figure()
		plt.plot(sample.time, robot_torque[:, 0], label='FL')
		plt.plot(sample.time, robot_torque[:, 1], label='FR')
		plt.plot(sample.time, robot_torque[:, 2], label='BL')
		plt.plot(sample.time, robot_torque[:, 3], label='BR')
		plt.title(f"torque {sample.name}")
		plt.legend()
		plt.show()
		import os
		if (os.path.isdir('/var/www/max/mpc_simulation')):
			plt.savefig(f"/var/www/max/mpc_simulation/torque_{sample.name.split('/')[-1]}.png")
			plt.close()

	if graph_motors:
		wheel_matrix = np.tile(R, (len(sample), 1, 1))
		measured_velocity = np.zeros((len(sample), 3)) # relative velocity
		measured_velocity[:, 0] = sample.x_velocity
		measured_velocity[:, 1] = sample.y_velocity
		measured_velocity[:, 2] = sample.angular_velocity

		measured_motor_velocity = np.einsum('ijk,ik->ij', wheel_matrix, measured_velocity)
		simulated_motor_velocity = np.einsum('ijk,ik->ij', wheel_matrix, simulated_velocity)

		plt.figure()

		plt.plot(sample.time, measured_motor_velocity[:, 0], label='FL (Measured)')
		plt.plot(sample.time, measured_motor_velocity[:, 1], label='FR (Measured)')
		plt.plot(sample.time, measured_motor_velocity[:, 2], label='BL (Measured)')
		plt.plot(sample.time, measured_motor_velocity[:, 3], label='BR (Measured)')

		plt.plot(sample.time, simulated_motor_velocity[:, 0], label='FL (Simulated)')
		plt.plot(sample.time, simulated_motor_velocity[:, 1], label='FR (Simulated)')
		plt.plot(sample.time, simulated_motor_velocity[:, 2], label='BL (Simulated)')
		plt.plot(sample.time, simulated_motor_velocity[:, 3], label='BR (Simulated)')

		plt.title(f"motors {sample.name}")
		plt.legend()
		plt.show()
		import os
		if (os.path.isdir('/var/www/max/mpc_simulation')):
			plt.savefig(f"/var/www/max/mpc_simulation/motors_{sample.name.split('/')[-1]}.png")
			plt.close()

	return np.sum(np.square(simulated_velocity[:, 2] - sample.angular_velocity)) + np.sum(
		np.square(simulated_velocity[:, 0] - sample.x_velocity)) + np.sum(
		np.square(simulated_velocity[:, 1] - sample.y_velocity))


@njit
def _simulate(x_position, y_position, angle, x_velocity, y_velocity, angular_velocity, battery_voltage, fl, fr, bl, br, motor_constant_e, motor_constant_t, armature_resistance, dynamic_friction, directional_friction, M_r, M_w):
	sample_cnt = len(x_position)

	start_pos = np.array([x_position[0], y_position[0], angle[0]])
	start_vel = np.array([x_velocity[0], y_velocity[0], angular_velocity[0]])

	robot_position = np.zeros((sample_cnt, 3))
	simulated_velocity = np.zeros((sample_cnt, 3))
	robot_torque = np.zeros((sample_cnt, 4))

	robot_position[0] = start_pos
	simulated_velocity[0] = start_vel
	for i in range(1, sample_cnt):
		voltage = battery_voltage[i - 1]
		powers = np.array([fl[i - 1], fr[i - 1], bl[i - 1], br[i - 1]])

		# dy/dt = [vel, accel]
		# f(tn, yn) = [simulated_velocity[i - 1], robot.acceleration(RobotState(RobotCommand(powers), robot_position[i - 1], simulated_velocity[i - 1]))]

		y_n = np.concatenate((robot_position[i - 1], simulated_velocity[i - 1]))

		def f(y):
			return np.concatenate((y[3:],
								   _acceleration(y[:3], y[3:], powers,
												 voltage, motor_constant_e, motor_constant_t,
												 armature_resistance, dynamic_friction, directional_friction, M_r, M_w
												 )
								   ))

		k1 = f(y_n)
		k2 = f(y_n + T / 2 * k1)
		k3 = f(y_n + T / 2 * k2)
		k4 = f(y_n + T * k3)

		y_n_plus_1 = y_n + T / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

		robot_position[i] = y_n_plus_1[:3]
		simulated_velocity[i] = y_n_plus_1[3:]
	psi = robot_position[:, 2]
	rotation_matrices = np.zeros((sample_cnt, 3, 3))
	rotation_matrices[:, 0, 0] = np.cos(-psi)
	rotation_matrices[:, 0, 1] = -np.sin(-psi)
	rotation_matrices[:, 1, 0] = np.sin(-psi)
	rotation_matrices[:, 1, 1] = np.cos(-psi)
	rotation_matrices[:, 2, 2] = 1
	# multiply each row of the robot velocity by the rotation matrix
	#simulated_velocity = np.einsum('ijk,ik->ij', rotation_matrices, simulated_velocity)
	for i in range(sample_cnt):
		simulated_velocity[i] = rotation_matrices[i] @ simulated_velocity[i]
		wheel_roller_velocity = R @ simulated_velocity[i]
		powers = np.array([fl[i], fr[i - 1], bl[i], br[i]])
		robot_torque[i] = _net_torque(wheel_roller_velocity, powers, voltage, motor_constant_e, motor_constant_t, armature_resistance, dynamic_friction)[:4]
	return robot_position, robot_torque, simulated_velocity


GRAD_STEP = .0001


def partial_derivative_jobs(objective, sample, args, parameter_name, pool: Pool) -> AsyncResult:
	args0 = args.copy()
	args1 = args.copy()

	args0[parameter_name] -= GRAD_STEP
	args1[parameter_name] += GRAD_STEP

	args_list = [(sample, DriveParameters.of_uniform_friction(**args0)), (sample, DriveParameters.of_uniform_friction(**args1))]

	def callback(result):
		return (result[1] - result[0]) / (2 * GRAD_STEP)

	# return promise and map
	return pool.starmap_async(objective, args_list, callback=callback)


def average_partial_derivative(samples, args, parameter_name, pool: Pool):
	jobs = [partial_derivative_jobs(simulate, sample, args, parameter_name, pool) for sample in samples]
	results = [job.get() for job in jobs]
	return np.mean(results, axis=0)


def grad(samples, args, param_names_grad, pool: Pool):
	grad = {}
	for param_name in param_names_grad:
		grad[param_name] = average_partial_derivative(samples, args, param_name, pool)
	return grad
