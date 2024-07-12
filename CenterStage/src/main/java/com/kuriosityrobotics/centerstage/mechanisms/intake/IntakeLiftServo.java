package com.kuriosityrobotics.centerstage.mechanisms.intake;

import static java.lang.Math.toRadians;

import com.kuriosityrobotics.centerstage.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLiftServo extends ServoControl {
	private static final double SERVO_SPEED = toRadians(180); // per second
	private static final double SERVO_RANGE = toRadians(150);
	public static final boolean FLIP_DIRECTION = true;
	private static final double ZERO_POSITION = toRadians(120);
	public IntakeLiftServo(Servo servo) throws InterruptedException {
		super(servo, SERVO_SPEED, SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
//		goToHeight(IntakeHeight.LIFTED);
	}

	public void goToHeight(IntakeHeight height) throws InterruptedException {
		goToAngle(height.position);
	}

	public enum IntakeHeight {
		LIFTED(toRadians(70)),
		STACK_FIVE(toRadians(10)),
		STACK_FOUR(toRadians(3)),
		GROUND(toRadians(-15));
		private final double position;
		IntakeHeight(double position) {
			this.position = position;
		}
	}
}
