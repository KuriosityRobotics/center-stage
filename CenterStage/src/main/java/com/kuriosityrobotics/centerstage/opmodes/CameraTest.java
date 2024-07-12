package com.kuriosityrobotics.centerstage.opmodes;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary;

import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.cameras.AprilTagOdometryAnalyzer;
import com.kuriosityrobotics.centerstage.cameras.AprilTagOdometryProcessor;
import com.kuriosityrobotics.centerstage.cameras.CameraNode;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Objects;

@TeleOp(name="Camera Test", group="Test")
public class CameraTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException{
		System.loadLibrary("apriltag");

		var ses = Robot.createScheduledExecutor();

		try {
			var hardwareProvider = new HardwareProviderImpl(hardwareMap);
			var bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);

			var tagProcessor = new AprilTagOdometryProcessor(AprilTagOdometryAnalyzer.RED_TEAM);

			CameraNode cameraNode = Robot.createCameraNode(ses, hardwareProvider, tagProcessor);
			var imu = Robot.createIMUNode(hardwareProvider);
			OdometryIntegrator odometry = new OdometryIntegrator(ses, imu, bulkDataFetcher, tagProcessor);

			Tester tester = new Tester(ses, telemetry, gamepad1);

			tester.header("Apriltag Odometry");
			waitForStart();
//			tester.instruct("Face the robot's intake towards the board (angle = 0)"); // ports don't work
			odometry.resetPosition(Pose.zero());
			tester.info("Odometry", () -> Objects.toString(odometry.getLocalisation().pose()));
			tester.info("Now point the camera at the tags; see if measurement works");

			while (opModeIsActive()){
				idle();
			}
		} finally {
			ses.shutdownNow();
		}
	}
}
