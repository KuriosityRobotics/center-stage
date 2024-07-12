package com.kuriosityrobotics.centerstage.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Do Nothing Auto", group = "Auto")
public class EmptyAuto extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		waitForStart();

		while (opModeIsActive()) {
			idle();
			// lol
		}
	}
}