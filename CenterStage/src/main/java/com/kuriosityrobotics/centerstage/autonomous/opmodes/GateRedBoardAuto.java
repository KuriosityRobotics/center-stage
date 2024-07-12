package com.kuriosityrobotics.centerstage.autonomous.opmodes;

import static com.kuriosityrobotics.centerstage.autonomous.RedAutoConstants.*;
import static com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeExtensionMotor.IntakeSlidePosition.FULL;

import com.kuriosityrobotics.centerstage.cameras.CameraNode;
import com.kuriosityrobotics.centerstage.cameras.GameElementAnalyzer;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Gate Red Board Auto", group="Gate Cycle")
@Disabled
public class GateRedBoardAuto extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try (var scope = HardwareTaskScope.open()) {
			HardwareProvider hardwareProvider = new HardwareProviderImpl(hardwareMap);

			var dt = Robot.createDrivetrainNode(hardwareProvider);

			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);
			var imu = Robot.createIMUNode(hardwareProvider);
			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);

			var follower = new MPCNode(ses, dt, odometryIntegrator, Robot.createStableVoltageSensor(ses, hardwareProvider));
			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);

			odometryIntegrator.resetPosition(BOARD_START_POSE);

			TeamElementProcessor teamElementProcessor = new TeamElementProcessor(GameElementAnalyzer.RED_BOARD);
			CameraNode cam = Robot.createCameraNode(ses, hardwareProvider, teamElementProcessor);

			waitForStart();

//			var follower = new MPCNode(ses, dt, odometryIntegrator, Robot.createStableVoltageSensor(ses, hardwareProvider));
//			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);

//			odometryIntegrator.resetPosition(FIELD_START_POSE);
//
//			System.out.println("AUTO: going to spike " + spike);
//
//			var pathToSpike = fieldToSpike(spike);
//
//			// go to spike
//			scope.fork(() -> follower.followPath(pathToSpike));
//			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
//			scope.join();
//			mechanisms.setIntakeSpeed(IntakeSpeed.REVERSE);
//			Thread.sleep(750);
//			mechanisms.setIntakeSpeed(IntakeSpeed.STOP);
//
//			// deposit
//			var toCorner = fieldSpikeToCorner(spike);
//			scope.fork(() -> follower.followPath(toCorner));
			scope.fork(() -> follower.followPath(BOARD_START_TO_BACKSTAGE));
			scope.fork(mechanisms::transfer);
			scope.join();

			scope.fork(() -> follower.followPath(BACKSTAGE_TO_BOARD));
			scope.fork(() -> mechanisms.toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.OVER_PASS));
			scope.join();

			scope.fork(() -> follower.followPath(BOARD_TO_BACKSTAGE));
			scope.fork(mechanisms::releaseAndCollapse);
			scope.join();

			scope.fork(() -> follower.followPath(BACKSTAGE_TO_CYCLE));
			Thread.sleep(1000);
			mechanisms.setIntakeSpeed(IntakeSpeed.FAST);
			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
			Thread.sleep(1500); // really bad

			mechanisms.extendIntake(FULL);
			scope.join();
			mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED);
			Thread.sleep(750);
			mechanisms.setIntakeSpeed(IntakeSpeed.REVERSE);

			// cycle 1
			scope.fork(() -> follower.followPath(CYCLE_TO_BACKSTAGE));
			scope.fork(mechanisms::transfer);
			Thread.sleep(1000);
			mechanisms.setIntakeSpeed(IntakeSpeed.REVERSE);
			scope.join();

			mechanisms.setIntakeSpeed(IntakeSpeed.STOP);
			scope.fork(() -> follower.followPath(BACKSTAGE_TO_BOARD));
			scope.fork(() -> mechanisms.toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.OVER_PASS));
			scope.join();

			scope.fork(() -> follower.followPath(BOARD_TO_BACKSTAGE));
			scope.fork(mechanisms::releaseAndCollapse);
			scope.join();

			scope.fork(() -> follower.followPath(BACKSTAGE_TO_CYCLE));
			Thread.sleep(1000);
			mechanisms.setIntakeSpeed(IntakeSpeed.FAST);
			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.GROUND));
			Thread.sleep(1500); // really bad

			mechanisms.extendIntake(FULL);
			scope.join();
			Thread.sleep(750);
			mechanisms.setIntakeSpeed(IntakeSpeed.REVERSE);

			// cycle 2
			scope.fork(() -> follower.followPath(CYCLE_TO_BACKSTAGE));
			scope.fork(mechanisms::transfer);
			Thread.sleep(1000);
			mechanisms.setIntakeSpeed(IntakeSpeed.REVERSE);
			scope.join();

			mechanisms.setIntakeSpeed(IntakeSpeed.STOP);
			scope.fork(() -> follower.followPath(BACKSTAGE_TO_BOARD));
			scope.fork(() -> mechanisms.toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.OVER_PASS));
			scope.join();

			scope.fork(() -> follower.followPath(BOARD_TO_BACKSTAGE));
			scope.fork(mechanisms::releaseAndCollapse);
			scope.join();
		} finally {
			ses.shutdownNow();
		}
	}
}