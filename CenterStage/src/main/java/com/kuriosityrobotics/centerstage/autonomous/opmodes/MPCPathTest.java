package com.kuriosityrobotics.centerstage.autonomous.opmodes;

import static java.lang.Math.toRadians;

import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeLiftServo;
import com.kuriosityrobotics.centerstage.mpc.DriveRecord;
import com.kuriosityrobotics.centerstage.mpc.MPCNode;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Objects;

@Autonomous(name="MPC Path Test", group="Test")
//@Disabled
public class MPCPathTest extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try {
			HardwareProvider hardwareProvider = new HardwareProviderImpl(hardwareMap);

			var dt = Robot.createDrivetrainNode(hardwareProvider);

			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);
			var imu = Robot.createIMUNode(hardwareProvider);
			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);

			var mpcFollower = new MPCNode(ses, dt, odometryIntegrator, Robot.createStableVoltageSensor(ses, hardwareProvider));
			var tester = new Tester(ses, telemetry, gamepad1);
			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);
			mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED);

			tester.header("MPC Path Test");
			tester.info("Position", () -> Objects.toString(odometryIntegrator.getGlobalLocalisation().pose()));

			waitForStart();

			tester.info("starting path 1");
//			mpcFollower.followPath(DriveRecord.zero());
//			mpcFollower.followPath(new DriveRecord.Builder(Pose.zero()).goTo(new Pose(0, 0, Math.toRadians(-90))).build());
			mpcFollower.followPath(new DriveRecord.Builder(Pose.zero())
				.goToQuickly(new Pose(1, 0, toRadians(360)))
				.build());

			tester.info("finished path 1");
			Thread.sleep(1000);
			tester.info("starting path 2");

			mpcFollower.followPath(new DriveRecord.Builder(new Pose(1, 0, toRadians(360)))
				.goTo(Pose.zero())
				.build());

			tester.info("finished path 2");

//			mpcFollower.followPath(new DriveRecord.Builder(Pose.zero())
//				.goTo(new Pose(0, 0, toRadians(20)))
//				.goTo(new Pose(0.2, 0.25, toRadians(20)))
//				.goTo(new Pose(0.8, 0.25, toRadians(-20)))
//				.goTo(new Pose(1.0, 0, toRadians(-20)))
//				.goTo(new Pose(1.2, -0.25, toRadians(-20)))
//				.goTo(new Pose(1.8, -0.25, toRadians(20)))
//				.goTo(new Pose(2.0, 0, toRadians(20)))
//				.goTo(new Pose(2.0, 0, toRadians(0)))
//				.build());
//
//			tester.info("finished path 1");
//			Thread.sleep(1000);
//			tester.info("starting path 2");
//
//			mpcFollower.followPath(new DriveRecord.Builder(new Pose(2.0, 0, toRadians(0)))
//				.goTo(new Pose(2.0, 0, toRadians(20)))
//				.goTo(new Pose(1.8, -0.25, toRadians(20)))
//				.goTo(new Pose(1.2, -0.25, toRadians(-20)))
//				.goTo(new Pose(1.0, 0, toRadians(-20)))
//				.goTo(new Pose(0.8, 0.25, toRadians(-20)))
//				.goTo(new Pose(0.2, 0.25, toRadians(20)))
//				.goTo(new Pose(0, 0, toRadians(20)))
//				.goTo(Pose.zero())
//				.build());

			while (opModeIsActive())
				idle();

		} finally {
			ses.shutdownNow();
		}
	}
}