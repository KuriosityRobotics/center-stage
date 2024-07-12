package com.kuriosityrobotics.centerstage.opmodes;

import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeLogger;
import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Outtake Test", group="Test")
@Disabled
public class OuttakeTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		var ses = Robot.createScheduledExecutor();

		try {
			HardwareProvider hardwareProvider = new HardwareProviderImpl(hardwareMap);
			BulkDataFetcher bulkDataFetcher = Robot.createBulkDataFetcher(ses, hardwareProvider);

			var mechanisms = Robot.createMechanismManager(hardwareProvider, bulkDataFetcher);

			waitForStart();

			OuttakeLogger logger = new OuttakeLogger(bulkDataFetcher);

//			outtake.releaseAndCollapse();
//			outtake.toDeposit();
//			outtake.releaseAndCollapse();

		} finally {
			ses.shutdownNow();
		}
	}
}