package com.kuriosityrobotics.centerstage.math;

import com.kuriosityrobotics.centerstage.util.Duration;

import java.io.Serializable;
import java.util.Objects;

import static java.lang.Math.toDegrees;

/**
 * This expresses velocity in free space broken into its linear and angular parts
 */
public class Twist implements Serializable {
	private static final Twist ZERO = new Twist(0, 0, 0);
	
	private final double x, y, angular;

	/**
	 * @param angular theta radians per second
	 */
	public Twist(double x, double y, double angular) {
		this.x = x;
		this.y = y;
		this.angular = angular;
	}

	public static Twist zero() {
		return ZERO;
	}

    public static Twist of(double xVel, double yVel, double thetaVel) {
		return new Twist(xVel, yVel, thetaVel);
    }

    public Pose apply(Pose pose, Duration duration) {
		var newPosition =
			pose.add(scalarMultiply(duration.toMillis() / 1000.));
		var newAngle = pose.orientation() + this.angular * (duration.toMillis() / 1000.);

		return new Pose(newPosition.x(), newPosition.y(), newAngle);
	}

	public Twist scalarMultiply(double a) {
		return new Twist(x() * a, y() * a, angular * a);
	}

	public double angular() {
		return angular;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == this) return true;
		if (obj == null || obj.getClass() != this.getClass()) return false;
		var that = (Twist) obj;
		return this.x() == that.x() &&
			this.y() == that.y() &&
			this.angular == that.angular;
	}

	public double x() {
		return x;
	}

	public double y() {
		return y;
	}

	public double getAngular() {
		return this.angular;
	}

	public Twist subtract(Twist other) {
		return new Twist(x() - other.x(), y() - other.y(), angular() - other.angular());
	}

	@Override
	public int hashCode() {
		return Objects.hash(x(), y(), angular);
	}

	@Override
	public String toString() {
		return String.format("[%.5f, %.5f, %.5f]", x(), y(), toDegrees(angular));
	}

	public double[] getData() {
		return new double[]{x(), y(), angular};
	}

	public double[] toArray() {
		return getData();
	}

	public Twist rotate(double angle) {
		return new Twist(
			x() * Math.cos(angle) - y() * Math.sin(angle),
			x() * Math.sin(angle) + y() * Math.cos(angle),
			angular
		);
	}

	public Twist add(Twist twist) {
		return Twist.of(
			this.x() + twist.x(),
			this.y() + twist.y(),
			this.angular() + twist.angular()
		);
	}

	public double velocity() {
		return Math.sqrt(x * x + y * y);
	}
}
