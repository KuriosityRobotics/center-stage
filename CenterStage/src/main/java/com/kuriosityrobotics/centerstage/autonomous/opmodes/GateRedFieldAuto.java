package com.kuriosityrobotics.centerstage.autonomous.opmodes;

import static com.kuriosityrobotics.centerstage.autonomous.BlueAutoConstants.fieldSpikeToGate;
import static com.kuriosityrobotics.centerstage.autonomous.BlueAutoConstants.fieldToSpikeForGate;

import com.kuriosityrobotics.centerstage.cameras.GameElementAnalyzer;
import com.kuriosityrobotics.centerstage.cameras.SpikeLocation;
import com.kuriosityrobotics.centerstage.cameras.TeamElementProcessor;
import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeLiftServo;
import com.kuriosityrobotics.centerstage.mpc.MPCNode;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.kuriosityrobotics.centerstage.autonomous.RedAutoConstants.FIELD_START_POSE;

@Autonomous(name="Gate Red Field Auto", group="Gate Cycle")
//@Disabled
public class GateRedFieldAuto extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try (var scope = HardwareTaskScope.open()) {
			HardwareProvider hardwareProvider = new HardwareProviderImpl(hardwareMap);

			var dt = Robot.createDrivetrainNode(hardwareProvider);
//			var tagProcessor = new AprilTagOdometryProcessor(AprilTagOdometryAnalyzer.RED_TEAM);

			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);
			var imu = Robot.createIMUNode(hardwareProvider);
//			var processor = new AprilTagOdometryProcessor(AprilTagOdometryAnalyzer.RED_TEAM);
			var teamElementProcessor = new TeamElementProcessor(GameElementAnalyzer.RED_FIELD);
			var cam = Robot.createCameraNode(ses, hardwareProvider, teamElementProcessor);
			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);

			var follower = new MPCNode(ses, dt, odometryIntegrator, Robot.createStableVoltageSensor(ses, hardwareProvider));
			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);

			odometryIntegrator.resetPosition(FIELD_START_POSE);

//			var tester = new Tester(ses, telemetry, gamepad1);
//			tester.header("camera");
//			tester.info("detection", () -> Objects.toString(teamElementProcessor.getSpot()));
//			tester.info("odo", () -> odometryIntegrator.getLocalisation().pose().toString());
			mechanisms.transfer();

			waitForStart();

			SpikeLocation spike = teamElementProcessor.awaitMeasurement();
			System.out.println("AUTO: going to spike " + spike);
			var pathToSpike = fieldToSpikeForGate(spike);
			mechanisms.toPurplePreload();

			scope.fork(() -> follower.followPath(pathToSpike));
			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
			scope.join();

			mechanisms.depositPurple();

//			var toCorner = fieldSpike(spike).mirror();
//			scope.fork(() -> follower.followPath(toCorner));
//			Pose spikePos = spikeToGate()
			follower.followPath(fieldSpikeToGate(spike));

			/* scope.fork(() -> follower.followPath(BACKSTAGE_TO_CYCLE));
			Thread.sleep(1000);
			mechanisms.setIntakeSpeed(IntakeSpeed.FAST);
			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
			Thread.sleep(1500); // really bad

			scope.fork(() -> mechanisms.extendIntake(FULL));
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

//			scope.fork(() -> follower.followPath(BOARD_TO_BACKSTAGE));
//			scope.fork(mechanisms::releaseAndCollapse);
//			scope.join(); */
		} finally {
			ses.shutdownNow();
		}
	}
}