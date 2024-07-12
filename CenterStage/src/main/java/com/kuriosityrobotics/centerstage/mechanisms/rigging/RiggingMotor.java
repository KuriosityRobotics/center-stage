package com.kuriosityrobotics.centerstage.mechanisms.rigging;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.CRServo;

public abstract class RiggingMotor {
	private final CRServo servo;

	public RiggingMotor(CRServo servo, boolean flip) {
		this.servo = servo;

		servo.setDirection(flip ? REVERSE : FORWARD);
		servo.setPower(-0.1);
	}

	public void raise() {
		servo.setPower(1);
	}

	public void lower() {
		servo.setPower(-1);
	}

	public void stop() {
		servo.setPower(0);
	}

	public static class Left extends RiggingMotor {
		public Left(CRServo servo) {
			super(servo, true);
		}
	}

	public static class Right extends RiggingMotor {
		public Right(CRServo servo) {
			super(servo, false);
		}
	}
}
