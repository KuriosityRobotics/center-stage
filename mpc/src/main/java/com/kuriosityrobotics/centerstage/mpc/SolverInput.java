package com.kuriosityrobotics.centerstage.mpc;

import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;

import java.util.Arrays;

import static com.kuriosityrobotics.centerstage.mpc.SolverOutput.*;

public class SolverInput {
	private final SystemState[] initialGuesses = new SystemState[NUM_STAGES];

	private final Pose startPose;
	private final Twist startVelocity;

	private final OptimisationParameters[] optimisationParameters = new OptimisationParameters[NUM_STAGES];

	public SolverInput(SystemState[] initialGuesses, Pose startPose, Twist startVelocity, OptimisationParameters[] optimisationParameters) {
		System.arraycopy(initialGuesses, 0, this.initialGuesses, 0, initialGuesses.length);
		this.startPose = startPose;
		this.startVelocity = startVelocity;
		System.arraycopy(optimisationParameters, 0, this.optimisationParameters, 0, optimisationParameters.length);
	}

	public static SolverInput ofStartingState(SystemState prevState, OptimisationParameters[] optimisationParameters) {
		var initialGuesses = new SystemState[NUM_STAGES];
		Arrays.fill(initialGuesses, prevState);
		var startPose = new Pose(prevState.getX(), prevState.getY(), prevState.getTheta());
		var startVelocity = new Twist(prevState.getXVel(), prevState.getYVel(), prevState.getThetaVel());

		return new SolverInput(initialGuesses, startPose, startVelocity, optimisationParameters);
	}

	public String toString() {
		var sb = new StringBuilder();
		for (var cond : initialGuesses) {
			sb.append("INITIAL");
			for (var number : cond.toArray()) {
				sb.append(" ").append(number);
			}
			sb.append("\n");
		}
		sb.append("XINIT ").append(startPose.x()).append(" ").append(startPose.y()).append(" ").append(startPose.orientation()).append(" ")
			.append(startVelocity.x()).append(" ").append(startVelocity.y()).append(" ").append(startVelocity.angular()).append("\n");
		for (var stage : optimisationParameters) {
			sb.append("STAGE");
			for (var number : stage.toArray()) {
				sb.append(" ").append(number);
			}
			sb.append("\n");
		}
		return sb.toString();
	}

	private double[] getOptimisationParametersArray() {
		var array = new double[OptimisationParameters.SIZE * NUM_STAGES];
		for (int i = 0; i < NUM_STAGES; i++) {
			System.arraycopy(optimisationParameters[i].toArray(), 0, array, i * OptimisationParameters.SIZE, OptimisationParameters.SIZE);
		}
		return array;
	}

	private double[] getInitialGuessesArray() {
		var array = new double[SystemState.SIZE * NUM_STAGES];
		for (int i = 0; i < NUM_STAGES; i++) {
			System.arraycopy(initialGuesses[i].toArray(), 0, array, i * SystemState.SIZE, SystemState.SIZE);
		}
		return array;
	}

	private double[] getStateArray() {
		// 6 long array of pose and twist
		var array = new double[6];
		System.arraycopy(startPose.toArray(), 0, array, 0, 3);
		System.arraycopy(startVelocity.toArray(), 0, array, 3, 3);
		return array;
	}

	static {
		System.loadLibrary("drivempc");
	}

	public native SolverOutput solve();
}
