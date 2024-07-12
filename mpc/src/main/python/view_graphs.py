from optuna_tune import initial_guess, samples
from simulation_cost import simulate

for sample in samples:
	simulate(sample, initial_guess, graph_motors=True, graph_velocity=True, graph_position=False)
	print(f"graphed {sample.name}")
