package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import com.kuriosityrobotics.centerstage.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.toRadians;

public class OuttakeArmLeveler extends ServoControl {
	private static final double ARM_SERVO_SPEED = toRadians(360); // per second
	private static final double ARM_SERVO_RANGE = toRadians(355);
	public static final boolean FLIP_DIRECTION = false;
	public static final double ZERO_POSITION = toRadians(95);

	public OuttakeArmLeveler(Servo servo) {
		super(servo, ARM_SERVO_SPEED, ARM_SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
	}
}
