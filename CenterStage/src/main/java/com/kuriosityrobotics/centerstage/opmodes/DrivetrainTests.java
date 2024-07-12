package com.kuriosityrobotics.centerstage.opmodes;

import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drivetrain Tester", group="Test")
@Disabled
public class DrivetrainTests extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();
		try {
			var hardwareProvider = new HardwareProviderImpl(hardwareMap);
			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);

			var dt = Robot.createDrivetrainNode(hardwareProvider);

			var imu = Robot.createIMUNode(hardwareProvider);
			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);

			var tester = new Tester(ses, telemetry, gamepad1);

			waitForStart();

			dt.testRoutine(tester);
			imu.testRoutine(tester, dt);
			odometryIntegrator.testRoutine(tester, dt);

			while (opModeIsActive()) {
				Thread.sleep(100);
			}
		} finally {
			ses.shutdownNow();
		}
	}
}