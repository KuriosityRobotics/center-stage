package com.kuriosityrobotics.centerstage.drive;

import java.io.Serializable;
import java.util.Objects;

/**
 * This class contains 4 motor powers.
 */
public final class MotorPowers implements Serializable {
	private final double frontLeft;
	private final double frontRight;
	private final double backLeft;
	private final double backRight;

	private MotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
		this.frontLeft = frontLeft;
		this.frontRight = frontRight;
		this.backLeft = backLeft;
		this.backRight = backRight;
	}

	public static MotorPowers ofPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
		return new MotorPowers(
			frontLeft,
			frontRight,
			backLeft,
			backRight
		);
	}

	public static MotorPowers zero() {
		return ofPowers(0, 0, 0, 0);
	}

	public double powerFrontLeft() {
		return frontLeft;
	}

	public double powerFrontRight() {
		return frontRight;
	}

	public double powerBackLeft() {
		return backLeft;
	}

	public double powerBackRight() {
		return backRight;
	}

	public MotorPowers scalarMultiply(double factor) {
		return new MotorPowers(
			frontLeft * factor,
			frontRight * factor,
			backLeft * factor,
			backRight * factor
		);
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == this) return true;
		if (obj == null || obj.getClass() != this.getClass()) return false;
		MotorPowers that = (MotorPowers) obj;
		return Double.doubleToLongBits(this.frontLeft) == Double.doubleToLongBits(that.frontLeft)
			&& Double.doubleToLongBits(this.frontRight)
			== Double.doubleToLongBits(that.frontRight)
			&& Double.doubleToLongBits(this.backLeft) == Double.doubleToLongBits(that.backLeft)
			&& Double.doubleToLongBits(this.backRight)
			== Double.doubleToLongBits(that.backRight);
	}

	@Override
	public int hashCode() {
		return Objects.hash(frontLeft, frontRight, backLeft, backRight);
	}

	@Override
	public String toString() {
		return "MotorVoltages["
			+ "frontLeft="
			+ ((int) (frontLeft * 100)) / 100.
			+ ", "
			+ "frontRight="
			+ ((int) (100 * frontRight)) / 100.
			+ ", "
			+ "backLeft="
			+ ((int) (100 * backLeft)) / 100.
			+ ", "
			+ "backRight="
			+ ((int) (100 * backRight)) / 100.
			+ ']';
	}

	public double[] toArray() {
		return new double[] {frontLeft, frontRight, backLeft, backRight};
	}
}
