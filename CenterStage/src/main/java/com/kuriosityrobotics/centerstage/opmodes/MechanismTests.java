package com.kuriosityrobotics.centerstage.opmodes;

import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mechanism Tester", group="Test")
@Disabled
public class MechanismTests extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try {
			var tester = new Tester(ses, telemetry, gamepad1);

			var hp = new HardwareProviderImpl(hardwareMap);
			var bdf = Robot.createBulkDataFetcher(ses, hp);

//			var intake = Robot.createIntake(hp);
//			var outtake = Robot.createOuttake(hp, bdf);
			var aeroplane = Robot.createAirplaneNode(hp);

			waitForStart();

//			intake.testRoutine(tester);
//			outtake.testRoutine(tester);
			aeroplane.testRoutine(tester);

			while (opModeIsActive()) {
				Thread.sleep(100);
			}
		} finally {
			ses.shutdownNow();
		}
	}
}
