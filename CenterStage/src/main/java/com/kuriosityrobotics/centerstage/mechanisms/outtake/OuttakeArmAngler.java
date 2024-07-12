package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import static java.lang.Math.toRadians;

import com.kuriosityrobotics.centerstage.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeArmAngler extends ServoControl {
	private static final double ARM_SERVO_SPEED = toRadians(360); // per second
	private static final double ARM_SERVO_RANGE = toRadians(355);
	public static final boolean FLIP_DIRECTION = false;
	public static final double ZERO_POSITION = toRadians(118);

	public OuttakeArmAngler(Servo servo) {
		super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
	}
}
