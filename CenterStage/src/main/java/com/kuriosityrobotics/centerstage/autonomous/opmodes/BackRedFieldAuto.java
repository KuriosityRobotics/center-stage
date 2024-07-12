package com.kuriosityrobotics.centerstage.autonomous.opmodes;

import static com.kuriosityrobotics.centerstage.autonomous.RedAutoConstants.*;
import com.kuriosityrobotics.centerstage.cameras.GameElementAnalyzer;
import com.kuriosityrobotics.centerstage.cameras.SpikeLocation;
import com.kuriosityrobotics.centerstage.cameras.TeamElementProcessor;
import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeLiftServo;
import com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeExtensionSlides;
import com.kuriosityrobotics.centerstage.mpc.MPCNode;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Back Red Field Auto", group="Backdoor Cycle")
public class BackRedFieldAuto extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try (var scope = HardwareTaskScope.open()) {
			HardwareProvider hardwareProvider = new HardwareProviderImpl(hardwareMap);

			var dt = Robot.createDrivetrainNode(hardwareProvider);

			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);
			var imu = Robot.createIMUNode(hardwareProvider);
			var teamElementProcessor = new TeamElementProcessor(GameElementAnalyzer.RED_FIELD);
			var cam = Robot.createCameraNode(ses, hardwareProvider, teamElementProcessor);

			var odometryIntegrator = new OdometryIntegrator(ses, imu, bulkDataFetcher);
			var follower = new MPCNode(ses, dt, odometryIntegrator, Robot.createStableVoltageSensor(ses, hardwareProvider));
			odometryIntegrator.resetPosition(FIELD_START_POSE);

			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);
			mechanisms.transfer();

			waitForStart();

			SpikeLocation spike = teamElementProcessor.awaitMeasurement();
			System.out.println("AUTO: going to spike " + spike);
//
			var pathToSpike = fieldToSpike(spike);
//
//			// go to spike
			mechanisms.toPurplePreload();
			scope.fork(() -> follower.followPath(pathToSpike));
			scope.fork(() -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
			scope.join();

			mechanisms.depositPurple();

			// ^ works

			var toCorner = fieldSpikeToCorner(spike);
			scope.fork(() -> follower.followPath(toCorner));

			// REAL
			scope.join();
			scope.fork(() -> follower.followPath(cornerToDeposit(spike)));
			scope.fork(() -> mechanisms.toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.RETRACTED));
			scope.join();

			mechanisms.flattenWrist();

			scope.fork(() -> follower.followPath(depositToCorner(spike)));
			scope.fork(mechanisms::releaseAndCollapse);
			scope.join();
		} finally {
			ses.shutdownNow();
		}
	}
}