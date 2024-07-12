package com.kuriosityrobotics.centerstage.mpc;

public class DriveWeights {
	public static final int SIZE = 7;
	public final double xWeight;
	public final double yWeight;
	public final double angleWeight;

	public final double xVelWeight;
	public final double yVelWeight;
	public final double angleVelWeight;

	public final double motorWeight;

	public DriveWeights(
		double xWeight, double yWeight, double angleWeight,
		double xVelWeight, double yVelWeight, double angleVelWeight,
		double motorWeight
	) {
		this.xWeight = xWeight;
		this.yWeight = yWeight;
		this.angleWeight = angleWeight;
		this.xVelWeight = xVelWeight;
		this.yVelWeight = yVelWeight;
		this.angleVelWeight = angleVelWeight;
		this.motorWeight = motorWeight;
	}

	public double[] toArray() {
		return new double[] {
			xWeight, yWeight, angleWeight,
			xVelWeight, yVelWeight, angleVelWeight,
			motorWeight
		};
	}
}
