package com.kuriosityrobotics.centerstage.mechanisms.airplane;

import com.kuriosityrobotics.centerstage.test.Tester;

public class AirplaneNode {
	private final AirplaneServo servo;

	public AirplaneNode(AirplaneServo servo) {
		this.servo = servo;
	}

	public void launchAirplane() throws InterruptedException {
		servo.launch();
	}

	public void testRoutine(Tester tester) throws InterruptedException {
		tester.header("[Aeroplane servo]");

		tester.instruct("Load aeroplane into launcher");
		launchAirplane();
		tester.confirmThat("aeroplane is launched");
	}
}
