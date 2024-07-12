package com.kuriosityrobotics.centerstage.mpc;

/**
 * This class holds three parameters:
 * {@link DriveParameters}, {@link DriveWeights}, and {@link DriveTargets}.
 * <p>
 * The array is ordered as follows:
 *     <li>{@link DriveParameters}</li>
 *     <li>{@link DriveWeights}</li>
 *     <li>{@link DriveTargets}</li>
 */
public class OptimisationParameters {
	public static final int SIZE = DriveParameters.SIZE + DriveWeights.SIZE + DriveTargets.SIZE;

	public final DriveParameters driveParameters;
	public final DriveWeights driveWeights;
	public final DriveTargets driveTargets;

	public OptimisationParameters(DriveParameters driveParameters, DriveWeights driveWeights, DriveTargets driveTargets) {
		this.driveParameters = driveParameters;
		this.driveWeights = driveWeights;
		this.driveTargets = driveTargets;
	}

	public double[] toArray() {
		double[] array = new double[SIZE];
		System.arraycopy(driveParameters.toArray(), 0, array, 0, DriveParameters.SIZE);
		System.arraycopy(driveWeights.toArray(), 0, array, DriveParameters.SIZE, DriveWeights.SIZE);
		System.arraycopy(driveTargets.toArray(), 0, array, DriveParameters.SIZE + DriveWeights.SIZE, DriveTargets.SIZE);
		return array;
	}
}
