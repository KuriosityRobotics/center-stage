package com.kuriosityrobotics.centerstage.mechanisms.airplane;

import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneServo {
	private final Servo servo;

	// 4 on the servo tester (original position)
	public AirplaneServo(Servo servo) {
		this.servo = servo;
		servo.setPosition(0);
	}

	// other side for launched
	public void launch() throws InterruptedException {
		servo.setPosition(1);
	}
}
