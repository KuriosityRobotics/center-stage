package com.kuriosityrobotics.centerstage.mpc;

public class DriveTargets {
	public static final int SIZE = 6;

	public final double x;
	public final double y;
	public final double angle;

	public final double xVel;
	public final double yVel;
	public final double angleVel;

	public DriveTargets(
		double x, double y, double angle,
		double xVel, double yVel, double angleVel
	) {
		this.x = x;
		this.y = y;
		this.angle = angle;
		this.xVel = xVel;
		this.yVel = yVel;
		this.angleVel = angleVel;
	}

	public static DriveTargets fromSystemState(SystemState state) {
		return new DriveTargets(
			state.getX(), state.getY(), state.getTheta(),
			state.getX(), state.getY(), state.getThetaVel()
		);
	}

	public double[] toArray() {
		return new double[] {
			x, y, angle,
			xVel, yVel, angleVel
		};
	}
}
