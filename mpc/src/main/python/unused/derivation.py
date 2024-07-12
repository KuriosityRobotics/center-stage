from sage.all import *

t = var('t')
robot_mass = var('m_r')
robot_moment = var('I_z')
wheel_moment, roller_moment = var('I_w, I_sigma')

def time_function(names):
	if isinstance(names, str):
		names = names.split(' ')
	return [function(name)(t) for name in names]
x, y, psi = time_function('x y psi')

q_r = matrix([x, y, psi]).T
M_r = diagonal_matrix([robot_mass, robot_mass, robot_moment])

M_w = diagonal_matrix([
	wheel_moment, wheel_moment, wheel_moment, wheel_moment,
	roller_moment, roller_moment, roller_moment, roller_moment
])

wheel_radius = var('r_w')
def R_row_angular_velocity(roller_angle, s, d):
	A = sin(roller_angle ) / (wheel_radius * sin(roller_angle))
	B = -cos(roller_angle ) / (wheel_radius * sin(roller_angle))
	C = (-d * sin(roller_angle) - s * cos(roller_angle )) / (wheel_radius * sin(roller_angle))
	return matrix([A, B, C])

def R_row_contact_point_velocity(roller_angle, s, d):
	a = 0
	b = 1 / sin(roller_angle)
	c = s / sin(roller_angle)
	return matrix([a, b, c])

L, l = var('L l')
front_left_angular_velocity = R_row_angular_velocity(pi/4, L, l)
front_right_angular_velocity = R_row_angular_velocity(-pi/4, L, -l)
back_left_angular_velocity = R_row_angular_velocity(-pi/4, -L, l)
back_right_angular_velocity = R_row_angular_velocity(pi/4, -L, -l)

front_left_contact_point_velocity = R_row_contact_point_velocity(pi/4, L, l)
front_right_contact_point_velocity = R_row_contact_point_velocity(-pi/4, L, -l)
back_left_contact_point_velocity = R_row_contact_point_velocity(-pi/4, -L, l)
back_right_contact_point_velocity = R_row_contact_point_velocity(pi/4, -L, -l)

R_ROWS = 8
R = block_matrix([
	front_left_angular_velocity,
	front_right_angular_velocity,
	back_left_angular_velocity,
	back_right_angular_velocity,

	front_left_contact_point_velocity,
	front_right_contact_point_velocity,
	back_left_contact_point_velocity,
	back_right_contact_point_velocity,
], nrows=R_ROWS)

robot_rotation = Matrix([
	[cos(psi), -sin(psi), 0],
	[sin(psi), cos(psi), 0],
	[0, 0, 1],
])

torques = matrix([
	var('tau_fl'), var('tau_fr'), var('tau_bl'), var('tau_br'),
	var('tau_sigma_fl'), var('tau_sigma_fr'), var('tau_sigma_bl'), var('tau_sigma_br')
]).T

H = M_r + robot_rotation * R.T * M_w * R * robot_rotation.T
K = robot_rotation * R.T * M_w * R * diff(robot_rotation, t).T
Fa = robot_rotation * R.T * torques

q_ddot = H.inverse() * (Fa - K * diff(q_r, t))

print(latex(q_ddot.simplify_full()))