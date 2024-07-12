package com.kuriosityrobotics.centerstage.mechanisms.intake;

import static java.lang.Math.toRadians;

import com.kuriosityrobotics.centerstage.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class IntakeContainerServo extends ServoControl {
	private static final double ARM_SERVO_SPEED = toRadians(150); // per second
	private static final double ARM_SERVO_RANGE = toRadians(360);

	protected IntakeContainerServo(Servo servo, boolean flipDirection, double zeroPosition) {
		super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, flipDirection, zeroPosition);
	}

	public static class Left extends IntakeContainerServo {
		public Left(Servo servo) {
			super(servo, true, toRadians(223));
		} // works
	}

	public static class Right extends IntakeContainerServo {
		public Right(Servo servo) {
			super(servo, false, toRadians(127));
		}
	}
}
