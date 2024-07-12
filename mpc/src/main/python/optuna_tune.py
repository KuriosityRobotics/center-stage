import glob

import matplotlib.pyplot as plt
import numpy as np

from simulation_cost import simulate
from drive_simulation import DriveParameters
from mecanum_data import DataSeries

samples = [DataSeries.from_csv(f) for f in glob.glob('drive_samples/r2/*.csv')]

initial_guess = DriveParameters(
	motor_constant_e=0.38869198809920497,
	motor_constant_t=0.17413942258926335,
	armature_resistance=0.9,
	robot_mass=12.999999983125175,
	robot_moment=0.01897086956915779,
	wheel_moment=0.006733058747799317,
	roller_moment=0.00010000000017017123,
	fl_wheel_friction=0.0,
	fr_wheel_friction=0.0,
	bl_wheel_friction=0.0,
	br_wheel_friction=0.0,
	fl_roller_friction=0.0,
	fr_roller_friction=0.0,
	bl_roller_friction=0.0,
	br_roller_friction=0.0,
	directional_friction_x=10.73229685481671,
	directional_friction_y=36.707741175403896,
	directional_friction_angle=7.347775273225521,
	battery_voltage=12,
)


def objective(model: DriveParameters):
	try:
		return np.sum([simulate(s, model) for s in samples])
	except Exception as e:
		print("EXCEPTION E",  e)
		return 1e10

if __name__ == '__main__':

	for sample in samples:
		simulate(sample, initial_guess, graph_motors=True, graph_velocity=True, graph_position=True)

	NUM_SEARCH_TRIALS = 200_000
	MAX_CONCURRENT = 128

	import optuna
	import ray
	from optuna.samplers import *
	from ray import tune
	from ray.tune import TuneConfig
	from ray.tune.search import ConcurrencyLimiter
	from ray.tune.search.optuna import OptunaSearch

	print("Optuna:", optuna.__version__, "Ray:", ray.__version__)
	ray.init(log_to_driver=False, ignore_reinit_error=True)

	sampler = CmaEsSampler(n_startup_trials=500)
	search_alg = OptunaSearch(sampler=sampler, metric="loss", mode="min",
							points_to_evaluate=[initial_guess.to_dict()])
	search_alg = ConcurrencyLimiter(search_alg, max_concurrent=MAX_CONCURRENT)
	tuner = tune.Tuner(
		lambda config: {"loss": objective(DriveParameters.of_uniform_friction(**config))},
		tune_config=TuneConfig(
			search_alg=search_alg,
			mode="min",
			num_samples=NUM_SEARCH_TRIALS,
		),
		run_config=ray.air.RunConfig(verbose=1),
		param_space={
			"motor_constant_e": tune.uniform(0.2, 0.4),
			"motor_constant_t": tune.uniform(0.15, 0.4),
			"armature_resistance": tune.uniform(0.9, 0.9),

			"robot_mass": tune.uniform(11, 13),
			"robot_moment": tune.uniform(0.01, 2),
			"wheel_moment": tune.uniform(0.001, 0.01),
			"roller_moment": tune.uniform(0.0001, 0.001),

			"fl_wheel_friction": tune.uniform(0., 0.0),
			"fr_wheel_friction": tune.uniform(0., 0.0),
			"bl_wheel_friction": tune.uniform(0., 0.0),
			"br_wheel_friction": tune.uniform(0., 0.0),
			"fl_roller_friction": tune.uniform(0., 0.0),
			"fr_roller_friction": tune.uniform(0., 0.0),
			"bl_roller_friction": tune.uniform(0., 0.0),
			"br_roller_friction": tune.uniform(0., 0.0),

			"directional_friction_x": tune.uniform(0., 50),
			"directional_friction_y": tune.uniform(0., 50),
			"directional_friction_angle": tune.uniform(0., 50),
		}
	)

	fit_results = tuner.fit()
	best_result = DriveParameters.of_uniform_friction(**fit_results.get_best_result('loss').config)
	df = fit_results.get_dataframe()

	plt.scatter(df.index[df['loss'] < 100], df['loss'][df['loss'] < 100])
	plt.title("Objective vs Iteration")
	plt.xlabel("Iteration")
	plt.ylabel("Objective")
	plt.show()

	print(f"Best hyperparameters found were {best_result!r}")
	for sample in samples:
		simulate(sample, best_result, graph_velocity=True, graph_position=False)

	print(f"best loss is {np.min(df['loss'])}")
