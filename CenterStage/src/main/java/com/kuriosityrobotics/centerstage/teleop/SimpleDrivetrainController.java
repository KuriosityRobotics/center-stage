package com.kuriosityrobotics.centerstage.teleop;

import static java.lang.Math.abs;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.mechanisms.DrivetrainNode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class SimpleDrivetrainController {
	private final Gamepad gamepad;
	private final DrivetrainNode drivetrainNode;

	public SimpleDrivetrainController(ScheduledExecutorService ses, DrivetrainNode drivetrainNode, Gamepad gamepad) {
		this.gamepad = gamepad;
		this.drivetrainNode = drivetrainNode;
		ses.scheduleWithFixedDelay(this::controllerPeriodic, 0, 1000 / 20, TimeUnit.MILLISECONDS);
	}

	public void controllerPeriodic() {
		drivetrainNode.setMotorVoltages(
			movementsToMotorVoltages(
				0.75 * Math.signum(-gamepad.left_stick_y) * Math.pow(abs(gamepad.left_stick_y), 2.),
				0.75 * Math.signum(-gamepad.left_stick_x) * Math.pow(abs(gamepad.left_stick_x), 2.),
				0.75 * Math.signum(-gamepad.right_stick_x) * Math.pow(abs(gamepad.right_stick_x), 1.))
		);
	}

	private MotorPowers movementsToMotorVoltages(double forwards, double sideways, double spin) {
		return MotorPowers.ofPowers(
			(forwards - sideways - spin),
			(forwards + sideways + spin),
			(forwards + sideways - spin),
			(forwards - sideways + spin));
	}
}
