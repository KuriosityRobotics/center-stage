package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import static com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeClaw.OuttakeClawState.*;
import static java.lang.Math.toRadians;

import com.kuriosityrobotics.centerstage.hardware.ServoControl;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeClaw extends ServoControl {
	public enum OuttakeClawState {
		LATCH(toRadians(0)),
		RELEASE(toRadians(110));

		final double angle;

		OuttakeClawState(double angle){ this.angle = angle; }
	}

	private static final double SERVO_SPEED = toRadians(720);
	private static final double SERVO_RANGE = toRadians(180);
	private static final boolean FLIP_DIRECTION = true;
	private static final double ZERO_POSITION = toRadians(180);

	public OuttakeClaw(Servo servo) throws InterruptedException {
		// both claws are identical
		super(servo, SERVO_SPEED, SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
		goToState(LATCH);
	}

	public void goToState(OuttakeClawState state) throws InterruptedException {
		goToAngle(state.angle);
	}

	public void latch() throws InterruptedException {
		goToState(LATCH);
	}

	public void unlatch() throws InterruptedException {
		goToState(RELEASE);
	}
}
