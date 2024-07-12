package com.kuriosityrobotics.centerstage.autonomous.opmodes;

import static com.kuriosityrobotics.centerstage.autonomous.BlueAutoConstants.*;
import static com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeExtensionMotor.IntakeSlidePosition.FULL;

import com.kuriosityrobotics.centerstage.cameras.CameraNode;
import com.kuriosityrobotics.centerstage.cameras.GameElementAnalyzer;
import com.kuriosityrobotics.centerstage.cameras.SpikeLocation;
import com.kuriosityrobotics.centerstage.cameras.TeamElementProcessor;
import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeLiftServo;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeSpeed;
import com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeExtensionSlides;
import com.kuriosityrobotics.centerstage.mpc.MPCNode;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Gate Blue Board Auto", group="Gate Cycle")
//@Disabled
public class GateBlueBoardAuto extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try (var scope = HardwareTaskScope.open()) {
			HardwareProvider hardwareProvider = new HardwareProviderImpl(hardwareMap);

			var dt = Robot.createDrivetrainNode(hardwareProvider);

			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);
			var imu = Robot.createIMUNode(hardwareProvider);
			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);
			var teamElementProcessor = new TeamElementProcessor(GameElementAnalyzer.BLUE_BOARD);
			CameraNode cam = Robot.createCameraNode(ses, hardwareProvider, teamElementProcessor);

			odometryIntegrator.resetPosition(BOARD_START_POSE.mirror());

			var follower = new MPCNode(ses, dt, odometryIntegrator, Robot.createStableVoltageSensor(ses, hardwareProvider));
			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);


//			var tester = new Tester(ses, telemetry, gamepad1);
//			tester.header("camera");
//			tester.info("detection", () -> Objects.toString(teamElementProcessor.getSpot()));
//			tester.info("odo", () -> odometryIntegrator.getLocalisation().pose().toString());

			mechanisms.transfer();

			waitForStart();

			SpikeLocation spike = teamElementProcessor.awaitMeasurement();
			System.out.println("AUTO: going to spike " + spike);
//
			var pathToSpike = boardToSpike(spike).mirror();
//
//			// go to spike
			mechanisms.toPurplePreload();
			scope.fork(() -> follower.followPath(pathToSpike));
			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
			scope.join();

			mechanisms.depositPurple();
			Thread.sleep(1000);

			// ^ works

//			var toCorner = boardSpikeToCorner(spike).mirror();
//			scope.fork(() -> follower.followPath(toCorner));

			var toDeposit = boardSpikeToDeposit(spike).mirror();
			scope.fork(() -> follower.followPath(toDeposit));
			scope.fork(() -> mechanisms.toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.RETRACTED));
			scope.join();

			mechanisms.flattenWrist();

			scope.fork(() -> follower.followPath(depositToBackstage(spike).mirror()));
			scope.fork(mechanisms::releaseAndCollapse);
			scope.join();

			// cycle 1

			scope.fork(() -> follower.followPath(BACKSTAGE_TO_CYCLE.mirror()));
			Thread.sleep(1000);
			mechanisms.setIntakeSpeed(IntakeSpeed.FAST);
			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
			scope.join();

			mechanisms.extendIntake(FULL);
			Thread.sleep(500);
			mechanisms.dropIntake();

			scope.fork(() -> follower.followPath(CYCLE_TO_BACKSTAGE.mirror()));
			scope.fork(mechanisms::transfer);
			mechanisms.setIntakeSpeed(IntakeSpeed.REVERSE);
			scope.join();

			mechanisms.setIntakeSpeed(IntakeSpeed.STOP);
			scope.fork(() -> follower.followPath(BACKSTAGE_TO_BOARD.mirror()));
			scope.fork(() -> mechanisms.toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.OVER_PASS));
			scope.join();

			scope.fork(() -> follower.followPath(BOARD_TO_BACKSTAGE.mirror()));
			scope.fork(mechanisms::releaseAndCollapse);
			scope.join();
		} finally {
			ses.shutdownNow();
		}
	}
}