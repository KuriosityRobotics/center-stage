package com.kuriosityrobotics.centerstage.mechanisms.intake;

import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;

import static com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeContainer.IntakeContainerPosition.INTAKE_READY;
import static com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeContainer.IntakeContainerPosition.TRANSFER;
import static com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeContainer.IntakeContainerPosition.TRANSFER_STEP;
import static java.lang.Math.toRadians;

public class IntakeContainer {
	public enum IntakeContainerPosition {
		INTAKE_READY(toRadians(-37)),
		TRANSFER_STEP(toRadians(120)),
		TRANSFER(toRadians(140));
		private final double position;

		IntakeContainerPosition(double pos) {
			this.position = pos;
		}
	}
	private final IntakeContainerServo right;
	private final IntakeContainerServo left;
	private final IntakeSensor sensor; // todo: use this

	private IntakeContainerPosition position;

	public IntakeContainer(IntakeContainerServo.Left left, IntakeContainerServo.Right right, IntakeSensor sensor) throws InterruptedException {
		this.right = right;
		this.left = left;
		this.sensor = sensor;

		goToPosition(INTAKE_READY);
	}

	public void goToPosition(IntakeContainerPosition position) throws InterruptedException {
		try (var scope = HardwareTaskScope.open()) {
			scope.fork(() -> left.goToAngle(position.position));
			scope.fork(() -> right.goToAngle(position.position));
			scope.join();
		}

		this.position = position;
	}

	public IntakeContainerPosition position() {
		return position;
	}

	public boolean containerFlipped() {
		return position == TRANSFER || position == TRANSFER_STEP;
	}

	public int numPixels() {
		return sensor.numPixels();
	}
}
