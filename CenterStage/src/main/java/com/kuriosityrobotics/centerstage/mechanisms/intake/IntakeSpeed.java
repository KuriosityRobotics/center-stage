package com.kuriosityrobotics.centerstage.mechanisms.intake;

public enum IntakeSpeed {
	STOP(0),
	FAST(1),
	SLOW(0.5),
	REVERSE(-0.9);

	final double power;

	IntakeSpeed(double power) {
		this.power = power;
	}
}
