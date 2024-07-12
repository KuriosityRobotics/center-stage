package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import static java.lang.Math.toRadians;

import androidx.annotation.GuardedBy;

import com.kuriosityrobotics.centerstage.hardware.ServoControl;
import com.kuriosityrobotics.centerstage.concurrent.PreemptibleLock;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeWrist extends ServoControl {
	private static final double SERVO_SPEED = toRadians(360);
	private static final double SERVO_RANGE = toRadians(180);
	private static final boolean FLIP_DIRECTION = false;
	private static final double ZERO_POSITION = toRadians(93.5);
	private final PreemptibleLock wristLock = new PreemptibleLock();
	@GuardedBy("wristLock")
	private WristAngle currentAngle = WristAngle.VERTICAL;

	public OuttakeWrist(Servo servo) throws InterruptedException {
		super(servo, SERVO_SPEED, SERVO_RANGE, FLIP_DIRECTION, ZERO_POSITION);
		toNeutral();
	}

	public void toHorizontal() throws InterruptedException {
		goToAngle(WristAngle.FLAT);
	}

	public void toNeutral() throws InterruptedException {
		goToAngle(WristAngle.VERTICAL);
	}

	public void incrementAngle() throws InterruptedException {
		if (currentAngle.ordinal() == WristAngle.values().length) throw new IllegalStateException("cannot increment angle");
		goToAngle(WristAngle.values()[currentAngle.ordinal() + 1]);
	}

	public void decrementAngle() throws InterruptedException {
		if (currentAngle.ordinal() == 0) throw new IllegalStateException("cannot decrement angle");
		goToAngle(WristAngle.values()[currentAngle.ordinal() - 1]);
	}

	public void goToAngle(WristAngle angle) throws InterruptedException {
		wristLock.lock();
		try {
			goToAngle(angle.angle);
			currentAngle = angle;
		} finally {
			wristLock.unlock();
		}
	}

	public enum WristAngle {
		FLAT(toRadians(-90)),
		SLANT_LEFT(toRadians(-30)),
		VERTICAL(toRadians(0)),
		SLANT_RIGHT(toRadians(45)),
		FLAT_RIGHT(toRadians(85));
		private final double angle;
		WristAngle(double angle) {
			this.angle = angle;
		}
	}
}
