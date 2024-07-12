package com.kuriosityrobotics.centerstage.mpc;

public class SolverOutput {
	public static final int NUM_STAGES = 5;

	private final int exitCode;
	private final SolutionInfo solutionInfo;
	private final SystemState[] states;

	public SolverOutput(int exitCode, SolutionInfo solutionInfo, SystemState[] states) {
		this.exitCode = exitCode;
		this.solutionInfo = solutionInfo;
		this.states = states;

		if (states.length != NUM_STAGES) {
			throw new IllegalArgumentException("states.length must be " + NUM_STAGES);
		}
	}

	public SolutionInfo getSolutionInfo() {
		return solutionInfo;
	}

	public SystemState[] getStates() {
		return states;
	}

	public int getExitCode() {
		return exitCode;
	}

	@Override
	public String toString() {
		var sb = new StringBuilder();
		sb.append("Exit Code: ").append(exitCode).append("\n");
		sb.append(solutionInfo).append("\n");
		sb.append("t ").append(SystemState.alignedHeader()).append("\n");
		for (int i = 0; i < NUM_STAGES; i++) {
			sb.append(i).append(" ").append(states[i]).append("\n");
		}

		return sb.toString();
	}

}
