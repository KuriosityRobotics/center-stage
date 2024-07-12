package com.kuriosityrobotics.centerstage.localisation;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.mechanisms.DrivetrainNode;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.kuriosityrobotics.centerstage.util.Instant;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.locks.ReentrantLock;

import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

public class CorrectedIMU {
	public double startAngle;
	private final IMU imu;

	public CorrectedIMU(IMU imu) {
		this.imu = imu;

		var logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
		var usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
		var orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

		imu.initialize(new IMU.Parameters(orientationOnRobot));
		imu.resetYaw();

		startAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
	}

	private double lastRawAngle = 0;
	private Instant lastAngleTime = Instant.now();
	private int fullRevolutionCount = 0;
	private final ReentrantLock angleLock = new ReentrantLock();

	public double getYaw() {
		angleLock.lock();
		try {
			double rawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
			if (Instant.now().since(lastAngleTime).toSeconds() <= .5) { // if our last reading was more than half a second ago, it's plausible that we've actually rotated
				if (rawAngle - lastRawAngle > PI)
					fullRevolutionCount--;
				else if (rawAngle - lastRawAngle < -PI)
					fullRevolutionCount++;
			}

			lastRawAngle = rawAngle;
			lastAngleTime = Instant.now();
			return startAngle + rawAngle + fullRevolutionCount * 2 * PI;
		} finally {
			angleLock.unlock();
		}
	}

	public double getYawVelocity() {
		return toRadians(imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate); // they don't know how to use radians
	}

	public void resetAngle(double newAngle) {
		angleLock.lock();
		try {
			this.startAngle = newAngle;
			this.lastAngleTime = Instant.now();
			this.lastRawAngle = 0;
			this.fullRevolutionCount = 0;
			imu.resetYaw();
		} finally {
			angleLock.unlock();
		}
	}

	private double wrapAngle(double angle) {
		angle %= 2 * PI;
		angle += 2 * PI;
		angle %= 2 * PI;
		return angle;
	}

	public void resetWrappedAngle(double angle) {
		angleLock.lock();
		try {
			double wrappedAngle = wrapAngle(angle);
			double angleDiff = wrappedAngle - startAngle;
            this.startAngle = wrappedAngle;
			if (angleDiff > 180) fullRevolutionCount++;
			if (angleDiff < -180) fullRevolutionCount--;

			this.lastAngleTime = Instant.now();
			this.lastRawAngle = 0;
			imu.resetYaw();
		} finally {
			angleLock.unlock();
		}
	}

	private static final double ANGLE_TOLERANCE = toRadians(2); // for position resets
	private static final double ANGLE_VEL_TOLERANCE = toRadians(5); // for spinning

	public void testRoutine(Tester tester, DrivetrainNode dt) throws InterruptedException {
		tester.header("[IMU]");
		tester.info("Angle (deg)", () -> String.format("%3.2f", toDegrees(getYaw())));

		dt.setMotorVoltages(MotorPowers.zero());
		tester.instruct("Place the robot flat on the ground");

		tester.automaticAssert("Is stationary?", getYawVelocity() <= ANGLE_VEL_TOLERANCE);

		tester.info("resetting angle to -90 deg");
		resetAngle(toRadians(-90));
		tester.info("current angle: " + toDegrees(getYaw()) + " deg");
		tester.automaticAssert("Reset angle: -90 deg", Math.abs(getYaw() - toRadians(-90)) <= ANGLE_TOLERANCE);

		tester.info("resetting angle to 720 deg");
		resetAngle(toRadians(720));
		tester.info("current angle: " + toDegrees(getYaw()) + " deg");
		tester.automaticAssert("Reset angle: 720 deg", Math.abs(getYaw() - toRadians(720)) <= ANGLE_TOLERANCE);

		tester.info("resetting angle to 0 deg");
		resetAngle(toRadians(0));
		tester.info("current angle: " + toDegrees(getYaw()) + " deg");
		tester.automaticAssert("Reset angle: 0 deg", Math.abs(getYaw()) <= ANGLE_TOLERANCE);

		tester.info("Spinning leftwards");
		dt.setMotorVoltages(MotorPowers.ofPowers(-0.2, +0.2, -0.2, 0.2));
		Thread.sleep(500);
		tester.automaticAssert("Is turning left?", getYawVelocity() >= ANGLE_VEL_TOLERANCE);
		tester.automaticAssert("Is facing left?", getYaw() >= ANGLE_TOLERANCE);
		tester.info("Stopping motors");
		dt.setMotorVoltages(MotorPowers.zero());
	}
}
