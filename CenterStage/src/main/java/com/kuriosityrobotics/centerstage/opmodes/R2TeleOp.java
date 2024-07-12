package com.kuriosityrobotics.centerstage.opmodes;

import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.kuriosityrobotics.centerstage.teleop.TeleopController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="TeleOp")
public class R2TeleOp extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();
		try {
			var hardwareProvider = new HardwareProviderImpl(hardwareMap);
			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);

			var dt = Robot.createDrivetrainNode(hardwareProvider);

			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);

			var aeroplane = Robot.createAirplaneNode(hardwareProvider);

			var rigging = Robot.createRiggingNode(hardwareProvider);

			var controller = new TeleopController(ses, gamepad1, gamepad2, dt, mechanisms, aeroplane, rigging);

			var imu = Robot.createIMUNode(hardwareProvider);
			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);

			waitForStart();

			while(opModeIsActive()){
				idle();
				// lol
			}
		} finally {
			ses.shutdownNow();
		}
	}
}