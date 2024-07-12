package com.kuriosityrobotics.centerstage.mechanisms.rigging;

public class RiggingNode {
	private final RiggingMotor leftServo;
	private final RiggingMotor rightServo;

	public RiggingNode(RiggingMotor.Left leftServo, RiggingMotor.Right rightServo) {
		this.leftServo = leftServo;
		this.rightServo = rightServo;
	}
	public void raise() {
		leftServo.raise();
		rightServo.raise();
	}

	public void raiseLeft() {
		leftServo.raise();
	}

	public void raiseRight() {
		rightServo.raise();
	}

	public void clasp() {
		leftServo.lower();
		rightServo.lower();
	}

	public void hold() {
		leftServo.stop();
		rightServo.stop();
	}
}
