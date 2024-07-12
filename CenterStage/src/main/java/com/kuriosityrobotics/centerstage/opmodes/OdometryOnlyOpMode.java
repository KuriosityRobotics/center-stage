package com.kuriosityrobotics.centerstage.opmodes;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.hardware.LynxHub;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.localisation.messages.LocalisationDatum;
import com.kuriosityrobotics.centerstage.mpc.DriveLogger;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.kuriosityrobotics.centerstage.teleop.SimpleDrivetrainController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.toDegrees;

@TeleOp(name="Drive Logger", group="Test")
public class OdometryOnlyOpMode extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try {
			HardwareProvider hardwareProvider = new HardwareProviderImpl(hardwareMap);

			var dt = Robot.createDrivetrainNode(hardwareProvider);

			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);
			var imu = Robot.createIMUNode(hardwareProvider);
			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);

//			var controller = new SimpleDrivetrainController(ses, dt, gamepad1); // remove if program drive powers

			waitForStart();

			var logger = new DriveLogger(ses, odometryIntegrator, dt, Robot.createStableVoltageSensor(ses, hardwareProvider), bulkDataFetcher.notifier(LynxHub.CONTROL_HUB));

			Thread.sleep(500);
			dt.setMotorVoltages(MotorPowers.ofPowers(-0.75, +0.75, -0.75, +0.75));
			Thread.sleep(1000);
			dt.setMotorVoltages(MotorPowers.zero());
			Thread.sleep(1500);
			dt.setMotorVoltages(MotorPowers.ofPowers(+0.75, -0.75, +0.75, -0.75));
			Thread.sleep(1000);
			dt.setMotorVoltages(MotorPowers.zero());
			Thread.sleep(1500);

			while (opModeIsActive()) idle();

//			while (opModeIsActive()) {
//				LocalisationDatum state = odometryIntegrator.getLocalisation();
//				if (state == null) continue;
//				telemetry.addData("angle", toDegrees(state.pose().orientation()));
//				telemetry.addData("angular vel", toDegrees(state.twist().angular()));
//				telemetry.addData("pose", state.pose());
//				telemetry.addData("relative twist", state.twist());
//				telemetry.addData("datum", state);
//				telemetry.addData("joystick y", gamepad1.left_stick_y);
//				telemetry.addData("joystick x", gamepad1.left_stick_x);
//				telemetry.update();
//				Thread.sleep(200);
//			}
		} finally {
			ses.shutdownNow();
		}
	}
}