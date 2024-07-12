package com.kuriosityrobotics.centerstage.mpc;

import com.kuriosityrobotics.centerstage.localisation.messages.LocalisationDatum;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;
import java.util.Arrays;

import static com.kuriosityrobotics.centerstage.mpc.SolverOutput.NUM_STAGES;

public class SolverInputBuilder {
	private final SystemState[] initialGuesses = new SystemState[NUM_STAGES];

	private Pose startPose;
	private Twist startVelocity;

	private final DriveParameters[] driveParameters = new DriveParameters[NUM_STAGES];
	private final DriveWeights[] driveWeights = new DriveWeights[NUM_STAGES];
	private final DriveTargets[] driveTargets = new DriveTargets[NUM_STAGES];


	public SolverInputBuilder() {
		Arrays.fill(initialGuesses, SystemState.from(1, 1, 1, 1, 0, 0, 0, 1, 1, 1));
		this.startPose = Pose.zero();
		this.startVelocity = Twist.zero();
	}

	public SolverInputBuilder(SolverInputBuilder input) {
		System.arraycopy(input.initialGuesses, 0, initialGuesses, 0, initialGuesses.length);
		this.startPose = input.startPose;
		this.startVelocity = input.startVelocity;
		System.arraycopy(input.driveParameters, 0, driveParameters, 0, driveParameters.length);
		System.arraycopy(input.driveWeights, 0, driveWeights, 0, driveWeights.length);
	}

	public SolverInputBuilder withInitialGuess(SystemState state) {
		Arrays.fill(initialGuesses, state);
		return this;
	}

	public SolverInputBuilder withPreviousSolution(SolverOutput output) {
		System.arraycopy(output.getStates(), 0, initialGuesses, 0, initialGuesses.length);
		return this;
	}

	public SolverInputBuilder withInitialGuesses(SystemState[] states) {
		System.arraycopy(states, 0, initialGuesses, 0, initialGuesses.length);
		return this;
	}

	public SolverInputBuilder setInitialGuessFor(int index, SystemState state) {
		initialGuesses[index] = state;
		return this;
	}

	public SolverInputBuilder startingAt(LocalisationDatum datum) {
		this.startPose = datum.pose();
		this.startVelocity = datum.twist();
		return this;
	}

	public SolverInputBuilder startingAt(SystemState state) {
		return startingAt(state.getLocalisation());
	}

	public SolverInputBuilder withDefaultParameters() {
		Arrays.fill(driveParameters, DriveParameters.ofDefaultDriveParameters(-1)); // battery voltage is set when build() is called
		return this;
	}

	public SolverInputBuilder withWeights(DriveWeights weights) {
		Arrays.fill(driveWeights, weights);
		return this;
	}

	public SolverInputBuilder withWeightsFor(int index, DriveWeights weights) {
		driveWeights[index] = weights;
		return this;
	}

	public SolverInputBuilder setTargetsFor(int index, DriveTargets targets) {
		driveTargets[index] = targets;
		return this;
	}

	public SolverInputBuilder setStageUsing(int index, SystemState state) {
		setInitialGuessFor(index, state);
		setTargetsFor(index, DriveTargets.fromSystemState(state));
		return this;
	}

	public SolverInput build(double batteryVoltage) {
		OptimisationParameters[] newStageParams = new OptimisationParameters[driveParameters.length];
		for (int i = 0; i < driveParameters.length; i++) {
			newStageParams[i] = new OptimisationParameters(
				DriveParameters.ofBatteryVoltage(batteryVoltage, driveParameters[i]),
				driveWeights[i],
				driveTargets[i]
			);
		}
		return new SolverInput(initialGuesses, startPose, startVelocity, newStageParams);
	}
}
