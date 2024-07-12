from dataclasses import dataclass

import numpy as np
import pandas as pd
from scipy import interpolate

T = .01


@dataclass
class DataSeries:
	name: str
	time: np.ndarray
	battery_voltage: np.ndarray
	x_position: np.ndarray
	y_position: np.ndarray
	angle: np.ndarray
	x_velocity: np.ndarray
	y_velocity: np.ndarray
	angular_velocity: np.ndarray
	x_acceleration: np.ndarray
	y_acceleration: np.ndarray
	angular_acceleration: np.ndarray
	fl: np.ndarray
	fr: np.ndarray
	bl: np.ndarray
	br: np.ndarray

	T = T

	def __post_init__(self):
		self.time = np.array(self.time)
		self.battery_voltage = np.array(self.battery_voltage)
		self.x_position = np.array(self.x_position)
		self.y_position = np.array(self.y_position)
		self.angle = np.array(self.angle)
		self.x_velocity = np.array(self.x_velocity)
		self.y_velocity = np.array(self.y_velocity)
		self.angular_velocity = np.array(self.angular_velocity)
		self.x_acceleration = np.array(self.x_acceleration)
		self.y_acceleration = np.array(self.y_acceleration)
		self.angular_acceleration = np.array(self.angular_acceleration)
		self.fl = np.array(self.fl)
		self.fr = np.array(self.fr)
		self.bl = np.array(self.bl)
		self.br = np.array(self.br)


	@staticmethod
	def _load_df(filename):
		df = pd.read_csv(filename)
		df['x_position'] = df['x_position']#.div(39.37) # We are in metric now, no need to convert from inch to metre
		df['y_position'] = df['y_position']#.div(39.37)
		df['x_velocity'] = df['x_velocity']#.div(39.37)
		df['y_velocity'] = df['y_velocity']#.div(39.37)
		df_resampled = pd.DataFrame()
		df_resampled['time'] = np.arange(0, df['time'].max(), step=T)
		for c in ['fl', 'fr', 'bl', 'br']:
			df_resampled[c] = interpolate.interp1d(df['time'], df[c], kind="previous", fill_value="extrapolate")(
				df_resampled['time'])
			df_resampled[c][np.isnan(df_resampled[c])] = 0

		for c in ['battery_voltage', 'x_position', 'y_position', 'angle', 'x_velocity', 'y_velocity',
				  'angular_velocity']:
			df_resampled[c] = interpolate.interp1d(df['time'], df[c], kind='cubic', fill_value="extrapolate")(
				df_resampled['time'])

		df_resampled['x_acceleration'] = np.gradient(df_resampled['x_velocity'], T)
		df_resampled['y_acceleration'] = np.gradient(df_resampled['y_velocity'], T)
		df_resampled['angular_acceleration'] = np.gradient(df_resampled['angular_velocity'], T)

		# clip powers to -1 to 1
		df_resampled['fl'] = np.clip(df_resampled['fl'], -1, 1)
		df_resampled['fr'] = np.clip(df_resampled['fr'], -1, 1)
		df_resampled['bl'] = np.clip(df_resampled['bl'], -1, 1)
		df_resampled['br'] = np.clip(df_resampled['br'], -1, 1)

		return df_resampled

	@staticmethod
	def from_csv(filename):
		df = DataSeries._load_df(filename)
		return DataSeries(filename, df['time'], df['battery_voltage'], df['x_position'], df['y_position'], df['angle'],
						  df['x_velocity'], df['y_velocity'], df['angular_velocity'], df['x_acceleration'],
						  df['y_acceleration'], df['angular_acceleration'],
						  df['fl'], df['fr'], df['bl'], df['br'])

	def plot(self, ax):
		ax.plot(self.x_position, self.y_position, label='Robot Path')
		ax.plot(self.x_position[0], self.y_position[0], 'ro', label='Start')
		ax.plot(self.x_position[-1], self.y_position[-1], 'go', label='End')
		ax.set_xlabel('X Position (m)')
		ax.set_ylabel('Y Position (m)')
		ax.legend()
		ax.grid()
		ax.set_aspect('equal')

	def plot_velocity(self, ax):
		ax.plot(self.time, self.x_velocity, label='X Velocity')
		ax.plot(self.time, self.y_velocity, label='Y Velocity')
		ax.plot(self.time, self.angular_velocity, label='Angular Velocity')
		ax.set_xlabel('Time (s)')
		ax.set_ylabel('Velocity (m/s)')
		ax.legend()
		ax.grid()

	def plot_acceleration(self, ax):
		ax.plot(self.time, self.x_acceleration, label='X Acceleration')
		ax.plot(self.time, self.y_acceleration, label='Y Acceleration')
		ax.plot(self.time, self.angular_acceleration, label='Angular Acceleration')
		ax.set_xlabel('Time (s)')
		ax.set_ylabel('Acceleration (m/s^2)')
		ax.legend()
		ax.grid()

	def plot_voltage(self, ax):
		ax.plot(self.time, self.battery_voltage, label='Battery Voltage')
		ax.set_xlabel('Time (s)')
		ax.set_ylabel('Voltage (V)')
		ax.legend()
		ax.grid()

	def plot_motor_powers(self, ax):
		ax.plot(self.time, self.fl, label='FL')
		ax.plot(self.time, self.fr, label='FR')
		ax.plot(self.time, self.bl, label='BL')
		ax.plot(self.time, self.br, label='BR')
		ax.set_xlabel('Time (s)')
		ax.set_ylabel('Motor Power')
		ax.legend()
		ax.grid()


	def __len__(self):
		return len(self.time)

	def __getitem__(self, item):
		if isinstance(item, slice):
			return DataSeries(self.name, self.time[item], self.battery_voltage[item], self.x_position[item],
							  self.y_position[item], self.angle[item], self.x_velocity[item], self.y_velocity[item],
							  self.angular_velocity[item], self.x_acceleration[item], self.y_acceleration[item],
							  self.angular_acceleration[item], self.fl[item], self.fr[item], self.bl[item],
							  self.br[item])
		else:
			return np.array([self.time[item], self.battery_voltage[item], self.x_position[item], self.y_position[item],
							 self.angle[item], self.x_velocity[item], self.y_velocity[item],
							 self.angular_velocity[item],
							 self.x_acceleration[item], self.y_acceleration[item], self.angular_acceleration[item],
							 self.fl[item], self.fr[item], self.bl[item], self.br[item]])
