import dataclasses
from dataclasses import dataclass

from numba import njit

if 'disabled' in njit.__dict__: # forces pro, use casadi
	import aerosandbox.numpy as np
else:
	import numpy as np


@njit
def rotation_matrix(psi):
	return np.array([[np.cos(psi), -np.sin(psi), 0.], [np.sin(psi), np.cos(psi), 0.], [0., 0., 1.]])


@njit
def rotation_matrix_derivative(psi, psidot):
	return np.array([[-np.sin(psi), -np.cos(psi), 0.], [np.cos(psi), -np.sin(psi), 0.], [0., 0., 0.]]) * psidot


def wheel_angular_velocity_R(roller_angle, wheel_radius, s, d):
	A = np.ones_like(roller_angle) # wheels are responsible for forwards (X) motion
	B = -np.cos(roller_angle) / np.sin(roller_angle)
	C = -d * A + s * B
	return np.stack([A, B, C], axis=-1) / wheel_radius


def roller_angular_velocity_R(roller_angle, roller_radius, s, d):
	a = np.zeros_like(roller_angle) # rollers are not involved in forwards (X) motion
	b = 1 / np.sin(roller_angle)
	c = -d * a + s * b
	return np.stack([a, b, c], axis=-1) / roller_radius

@dataclass
class RobotCommand:
	powers: np.ndarray

	@property
	def fl(self):
		return self.powers[0]

	@property
	def fr(self):
		return self.powers[1]

	@property
	def bl(self):
		return self.powers[2]

	@property
	def br(self):
		return self.powers[3]

	def to_array(self):
		return np.array([self.fl, self.fr, self.bl, self.br])

	@staticmethod
	def from_array(array):
		return RobotCommand(array)

	@staticmethod
	def num_parameters():
		return 4

@dataclass
class RobotState:
	command: RobotCommand
	position: np.ndarray
	velocity: np.ndarray

	@property
	def x(self):
		return self.position[0]

	@property
	def y(self):
		return self.position[1]

	@property
	def angle(self):
		return self.position[2]

	@property
	def vx(self):
		return self.velocity[0]

	@property
	def vy(self):
		return self.velocity[1]

	@property
	def vangle(self):
		return self.velocity[2]

	def to_array(self):
		return np.concatenate((self.command.to_array(), self.position, self.velocity))

	@staticmethod
	def from_array(array):
		return RobotState(RobotCommand.from_array(array[:4]), array[4:7], array[7:])


ROBOT_FORWARDS_AXIS: float = 0.214 / 2 # half
ROBOT_SIDEWAYS_AXIS: float = 0.289 / 2 # half
WHEEL_RADIUS: float = 0.048
ROLLER_RADIUS: float = 0.0097

d = np.array([1, -1, 1, -1]) * ROBOT_SIDEWAYS_AXIS
s = np.array([1, 1, -1, -1]) * ROBOT_FORWARDS_AXIS
roller_angles = np.array([np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4])

wheel_angular_velocity_rows = wheel_angular_velocity_R(roller_angles, WHEEL_RADIUS, s, d)
roller_angular_velocity_rows = roller_angular_velocity_R(roller_angles, ROLLER_RADIUS, s, d)
R = np.vstack((wheel_angular_velocity_rows, roller_angular_velocity_rows))

@njit
def _applied_torque(wheel_roller_velocity, powers, voltage, motor_constant_e, motor_constant_t, armature_resistance):
	motor_velocity = wheel_roller_velocity[:4] # no gearing: motor velocity = wheel velocity
	applied_voltage = voltage * powers # assumption: proportional to power and battery voltage
	back_emf_voltage = motor_velocity * motor_constant_e # induced voltage opposes motion
	effective_voltage = applied_voltage - back_emf_voltage
	motor_current = effective_voltage / armature_resistance # Ohm's law
	applied_motor_torque = motor_current * motor_constant_t # no gearing: wheel torque = motor torque
	return np.concatenate((applied_motor_torque, np.zeros(4))) # motors don't apply torque to rollers


@njit
def _friction_torque(wheel_roller_velocity, dynamic_friction):
	return np.sign(wheel_roller_velocity) * dynamic_friction


@njit
def _net_torque(wheel_roller_velocity, powers, voltage, motor_constant_e, motor_constant_t, armature_resistance, dynamic_friction):
	applied_torque = _applied_torque(wheel_roller_velocity, powers, voltage, motor_constant_e, motor_constant_t, armature_resistance)
	friction_torque = _friction_torque(wheel_roller_velocity, dynamic_friction)

	net_torque = applied_torque - friction_torque
	return net_torque


@njit
def _acceleration(position, velocity, powers, voltage, motor_constant_e, motor_constant_t, armature_resistance, dynamic_friction, directional_friction, M_r, M_w):
	angle = position[2]
	angular_velocity = velocity[2]

	rotation = rotation_matrix(angle)
	rotation_dot = rotation_matrix_derivative(angle, angular_velocity)

	wheel_roller_velocity = R @ rotation.T @ velocity

	H = M_r + rotation @ R.T @ M_w @ R @ rotation.T
	K = rotation @ R.T @ M_w @ R @ rotation_dot.T
	F_a = rotation @ (R.T @ _net_torque(wheel_roller_velocity, powers, voltage, motor_constant_e, motor_constant_t, armature_resistance, dynamic_friction) - np.sign(rotation.T @ velocity) * directional_friction)
	acceleration = np.linalg.inv(H) @ (F_a - K @ velocity)
	return acceleration


@dataclass
class DriveParameters:
	motor_constant_e: float = 0.33570545232395443
	motor_constant_t: float = 0.19340444224622078
	armature_resistance: float = 0.9

	robot_mass: float = 12.33323012127017
	robot_moment: float = 0.1315328580572256
	wheel_moment: float = 0.00668203785816994
	roller_moment: float = 0.00010370537790609307

	fl_wheel_friction: float = 0.
	fr_wheel_friction: float = 0.
	bl_wheel_friction: float = 0.
	br_wheel_friction: float = 0.

	fl_roller_friction: float = 0.
	fr_roller_friction: float = 0.
	bl_roller_friction: float = 0.
	br_roller_friction: float = 0.

	directional_friction_x: float = 3.3296478994755456
	directional_friction_y: float = 24.9603707710718
	directional_friction_angle: float = 3.183599056707547

	battery_voltage: float = 12

	@staticmethod
	def of_uniform_friction(**kwargs):
		if 'roller_friction' in kwargs:
			friction = kwargs['roller_friction']
			kwargs['fl_roller_friction'] = friction
			kwargs['fr_roller_friction'] = friction
			kwargs['bl_roller_friction'] = friction
			kwargs['br_roller_friction'] = friction
			del kwargs['roller_friction']

		if 'wheel_friction' in kwargs:
			friction = kwargs['wheel_friction']
			kwargs['fl_wheel_friction'] = friction
			kwargs['fr_wheel_friction'] = friction
			kwargs['bl_wheel_friction'] = friction
			kwargs['br_wheel_friction'] = friction
			del kwargs['wheel_friction']

		return DriveParameters(**kwargs)


	def to_dict(self):
		result = dataclasses.asdict(self)
		del result['battery_voltage']
		return result


	def to_uniform_friction_dict(self):
		"""
		Converts the drive model to a dictionary with uniform wheel friction
		"""
		if not self.is_uniform_friction():
			raise ValueError('DriveModel does not have uniform wheel friction')

		result = dataclasses.asdict(self)
		result['roller_friction'] = result['fl_roller_friction']
		del result['fl_roller_friction']
		del result['fr_roller_friction']
		del result['bl_roller_friction']
		del result['br_roller_friction']

		result['wheel_friction'] = result['fl_wheel_friction']
		del result['fl_wheel_friction']
		del result['fr_wheel_friction']
		del result['bl_wheel_friction']
		del result['br_wheel_friction']

		result['directional_friction'] = result['directional_friction_x']
		del result['directional_friction_x']
		del result['directional_friction_y']
		del result['directional_friction_angle']

		del result['battery_voltage']

		return result

	def to_array(self):
		return np.array([
			self.motor_constant_e,
			self.motor_constant_t,
			self.armature_resistance,
			self.robot_mass,
			self.robot_moment,
			self.wheel_moment,
			self.roller_moment,
			self.fl_wheel_friction,
			self.fr_wheel_friction,
			self.bl_wheel_friction,
			self.br_wheel_friction,
			self.fl_roller_friction,
			self.fr_roller_friction,
			self.bl_roller_friction,
			self.br_roller_friction,
			self.directional_friction_x,
			self.directional_friction_y,
			self.directional_friction_angle,
			self.battery_voltage
		])

	@staticmethod
	def from_array(array):
		return DriveParameters(
			motor_constant_e=array[0],
			motor_constant_t=array[1],
			armature_resistance=array[2],
			robot_mass=array[3],
			robot_moment=array[4],
			wheel_moment=array[5],
			roller_moment=array[6],
			fl_wheel_friction=array[7],
			fr_wheel_friction=array[8],
			bl_wheel_friction=array[9],
			br_wheel_friction=array[10],
			fl_roller_friction=array[11],
			fr_roller_friction=array[12],
			bl_roller_friction=array[13],
			br_roller_friction=array[14],
			directional_friction_x=array[15],
			directional_friction_y=array[16],
			directional_friction_angle=array[17],
			battery_voltage=array[18]
		)

	def is_uniform_friction(self):
		"""
		Checks if the drive model has uniform wheel and roller friction
		:return: True if the drive model has uniform wheel and roller friction, False otherwise
		"""
		return (
			self.fl_roller_friction == self.fr_roller_friction == self.bl_roller_friction == self.br_roller_friction and
			self.fl_wheel_friction == self.fr_wheel_friction == self.bl_wheel_friction == self.br_wheel_friction and
			self.directional_friction_x == self.directional_friction_y == self.directional_friction_angle
		)

	def __post_init__(self):
		self.M_r = np.diag(np.array([self.robot_mass, self.robot_mass, self.robot_moment]))
		self.M_w = np.diag(np.concatenate((
			np.ones(4) * self.wheel_moment,
			np.ones(4) * self.roller_moment
		)))

		self.dynamic_friction = np.array([
			self.fl_wheel_friction,
			self.fr_wheel_friction,
			self.bl_wheel_friction,
			self.br_wheel_friction,

			self.fl_roller_friction,
			self.fr_roller_friction,
			self.bl_roller_friction,
			self.br_roller_friction
		])

		self.directional_friction = np.array([
			self.directional_friction_x,
			self.directional_friction_y,
			self.directional_friction_angle
		])

	def acceleration(self, state: RobotState):
		return _acceleration(
			state.position,
			state.velocity,
			state.command.powers,
			self.battery_voltage,
			self.motor_constant_e,
			self.motor_constant_t,
			self.armature_resistance,
			self.dynamic_friction,
			self.directional_friction,
			self.M_r,
			self.M_w
		)

	def net_torque(self, state: RobotState):
		return _net_torque(
			R @ rotation_matrix(state.angle).T @ state.velocity,
			state.command.powers,
			self.battery_voltage,
			self.motor_constant_e,
			self.motor_constant_t,
			self.armature_resistance,
			self.dynamic_friction
		)

	def continuous_dynamics(self, state: RobotState):
		velocity = state.velocity
		acceleration = self.acceleration(state)
		return np.concatenate((velocity, acceleration))

	def __repr__(self):
		"""
		prettier repr
		:return:
		"""
		result = f"{self.__class__.__name__}(\n"
		for field in dataclasses.fields(self):
			result += f"\t{field.name}={getattr(self, field.name)},\n"

		result += ")"
		return result

	@staticmethod
	def num_parameters():
		return 19

@dataclass
class DriveWeights:
	position_weight: np.ndarray
	velocity_weight: np.ndarray
	motor_weight: float

	@staticmethod
	def num_parameters():
		return 7

	@property
	def x_weight(self):
		return self.position_weight[0]

	@property
	def y_weight(self):
		return self.position_weight[1]

	@property
	def angle_weight(self):
		return self.position_weight[2]

	@property
	def vx_weight(self):
		return self.velocity_weight[0]

	@property
	def vy_weight(self):
		return self.velocity_weight[1]

	@property
	def vangle_weight(self):
		return self.velocity_weight[2]

	def to_array(self):
		return np.concatenate((self.position_weight, self.velocity_weight, np.array([self.motor_weight])))

	def to_state_array(self):
		return np.concatenate((np.ones(4) * self.motor_weight, self.position_weight, self.velocity_weight))

	@staticmethod
	def from_array(array):
		return DriveWeights(
			position_weight=array[:3],
			velocity_weight=array[3:6],
			motor_weight=array[6]
		)

@dataclass
class DriveTargets:
	position_target: np.ndarray
	velocity_target: np.ndarray

	@staticmethod
	def num_parameters():
		return 6

	def to_array(self):
		return np.concatenate((self.position_target, self.velocity_target))

	def to_state_array(self):
		return np.concatenate((np.zeros(4), self.position_target, self.velocity_target))

	@staticmethod
	def from_array(array):
		return DriveTargets(
			position_target=array[:3],
			velocity_target=array[3:]
		)

@dataclass
class OptimisationParameters:
	parameters: DriveParameters
	weights: DriveWeights
	targets: DriveTargets

	def least_squares_objective(self, state: RobotState):
		"""
		The objective function to be minimised by the optimiser
		:param state: the current state of the robot
		:return: the objective function deviation, which will be squared and summed
		"""
		state_array = state.to_array()
		target_array = self.targets.to_state_array()
		weight_array = self.weights.to_state_array()

		return (state_array - target_array) * weight_array

	def objective(self, state: RobotState):
		"""
		The objective function to be minimised by the optimiser
		:param state: the current state of the robot
		:return: the objective function deviation, which will be squared and summed
		"""
		error_array = self.least_squares_objective(state)
		return np.sum(np.square(error_array))

	@staticmethod
	def num_parameters():
		return DriveParameters.num_parameters() + DriveWeights.num_parameters() + DriveTargets.num_parameters()

	def to_array(self):
		return np.concatenate((
			self.parameters.to_array(),
			self.weights.to_array(),
			self.targets.to_array()
		))

	@staticmethod
	def from_array(array):
		return OptimisationParameters(
			parameters=DriveParameters.from_array(array[:DriveParameters.num_parameters()]),
			weights=DriveWeights.from_array(array[DriveParameters.num_parameters():DriveParameters.num_parameters() + DriveWeights.num_parameters()]),
			targets=DriveTargets.from_array(array[DriveParameters.num_parameters() + DriveWeights.num_parameters():])
		)