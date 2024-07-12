package com.kuriosityrobotics.centerstage.mechanisms.intake;

import java.util.concurrent.TimeoutException;

public class Intake {
	private final IntakeMotor motor;
	private final IntakeExtensionMotor slides;
	private final IntakeContainer container;
	private final IntakeLiftServo lift;

	public Intake(IntakeMotor motor, IntakeExtensionMotor slides, IntakeContainer container, IntakeLiftServo lift){
		this.motor = motor;
		this.slides = slides;
		this.container = container;
		this.lift = lift;
	}

	public void calibrate() throws InterruptedException {
		slides.calibrate();
	}

	public void setIntakeSpeed(IntakeSpeed speed) {
		motor.setPower(speed);
	}

	public void flip() throws InterruptedException {
		container.goToPosition(IntakeContainer.IntakeContainerPosition.TRANSFER);
	}

	public void flipStep() throws InterruptedException {
		container.goToPosition(IntakeContainer.IntakeContainerPosition.TRANSFER_STEP);
	}

	public void unflip() throws InterruptedException {
		container.goToPosition(IntakeContainer.IntakeContainerPosition.INTAKE_READY);
		slides.ensureMotorsEngaged();
	}

	public void goTo(IntakeExtensionMotor.IntakeSlidePosition position) throws InterruptedException, TimeoutException {
		slides.goToPosition(position);
	}

	public void retractAndLock() throws InterruptedException, TimeoutException {
		slides.retractAndLock();
	}

	public boolean containerFlipped() {
		return container.containerFlipped();
	}

	public void goToHeight(IntakeLiftServo.IntakeHeight height) throws InterruptedException {
		lift.goToHeight(height);
	}

	public int numPixels() {
		return container.numPixels();
	}
}