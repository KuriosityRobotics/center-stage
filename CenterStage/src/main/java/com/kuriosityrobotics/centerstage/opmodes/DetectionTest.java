package com.kuriosityrobotics.centerstage.opmodes;

import com.kuriosityrobotics.centerstage.cameras.CameraNode;
import com.kuriosityrobotics.centerstage.cameras.GameElementAnalyzer;
import com.kuriosityrobotics.centerstage.cameras.TeamElementProcessor;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Detection Test", group="Test")
@Disabled
public class DetectionTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException{
		var ses = Robot.createScheduledExecutor();

		try {
			var hardwareProvider = new HardwareProviderImpl(hardwareMap);
			var processor = new TeamElementProcessor(GameElementAnalyzer.RED_BOARD);

			CameraNode cameraNode = Robot.createCameraNode(ses, hardwareProvider, processor);

			waitForStart();
			while (opModeIsActive()) {
//				telemetry.addData("Element on Line", String.format("%s\n", processor.getSpot()));
//				telemetry.update();
			}
		} finally {
			ses.shutdownNow();
		}
	}
}
