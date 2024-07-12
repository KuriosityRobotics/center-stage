package com.kuriosityrobotics.centerstage.mpc;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.localisation.messages.LocalisationDatum;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;

public class SystemState {
	public static final int SIZE = 10;
	private final double fl;
	private final double fr;
	private final double bl;
	private final double br;

	private final double x;
	private final double y;
	private final double theta;

	private final double xVel;
	private final double yVel;
	private final double thetaVel;

	private SystemState(
		double fl, double fr, double bl, double br,
		double x, double y, double theta,
		double xVel, double yVel, double thetaVel
	) {
		this.fl = fl;
		this.fr = fr;
		this.bl = bl;
		this.br = br;
		this.x = x;
		this.y = y;
		this.theta = theta;
		this.xVel = xVel;
		this.yVel = yVel;
		this.thetaVel = thetaVel;
	}

	public static SystemState from(
		double fl, double fr, double bl, double br,
		double x, double y, double theta,
		double xVel, double yVel, double thetaVel
	) {
		return new SystemState(
			fl, fr, bl, br,
			x, y, theta,
			xVel, yVel, thetaVel
		);
	}

	public static SystemState fromDoubleArray(double[] array) {
        return new SystemState(
            array[0], array[1], array[2], array[3],
            array[4], array[5], array[6],
            array[7], array[8], array[9]
        );
    }

	public static SystemState ofLocalisationAndPowers(LocalisationDatum localisation, MotorPowers powers) {
		return SystemState.from(
			powers.powerFrontLeft(), powers.powerFrontRight(), powers.powerBackLeft(), powers.powerBackRight(),
			localisation.pose().x(), localisation.pose().y(), localisation.pose().orientation(),
			localisation.twist().x(), localisation.twist().y(), localisation.twist().angular()
		);
	}

	public double[] toArray() {
		return new double[] {
			fl, fr, bl, br,
			x, y, theta,
			xVel, yVel, thetaVel
		};
	}

	public double getFl() { return fl; }
	public double getFr() { return fr; }
	public double getBl() { return bl; }
	public double getBr() { return br; }

	public double getX() { return x; }
	public double getY() { return y; }
	public double getTheta() { return theta; }

	public double getXVel() { return xVel; }
	public double getYVel() { return yVel; }
	public double getThetaVel() { return thetaVel; }


	@Override
	public String toString() {
		return String.format("MotorPowers(%04.3f, %04.3f, %04.3f, %04.3f), position(%04.3f %04.3f %04.3f), velocity(%04.3f %04.3f %04.3f)",
			fl, fr, bl, br,
			x, y, theta,
			xVel, yVel, thetaVel
		);
	}

	public static String alignedHeader() {
		return String.format("            %-5s  %-5s  %-5s  %-5s              %-5s %-5s %-5s             %-5s %-5s %-5s",
			"fl", "fr", "bl", "br",
			"x", "y", "Θ",
			"u", "v", "Φ"
		);
	}

	public MotorPowers getMotorPowers() {
		return MotorPowers.ofPowers(fl, fr, bl, br);
	}

	public LocalisationDatum getLocalisation() {
		return LocalisationDatum.of(
			Pose.of(x, y, theta),
			Twist.of(xVel, yVel, thetaVel)
		);
	}

	public double linearDistanceTo(SystemState other) {
		return Math.sqrt(Math.pow(other.x - x, 2) + Math.pow(other.y - y, 2));
	}

	/**
	 * Mirrors the SystemState across the X-axis,
	 * negating the Y and Theta position and angle values.
	 */
	public SystemState mirror() {
		return new SystemState(
			fl, fr, bl, br,
			x, -y, -theta,
			xVel, -yVel, -thetaVel
		);
	}
}
