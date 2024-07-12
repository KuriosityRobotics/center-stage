package com.kuriosityrobotics.centerstage.opmodes;

import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.hardware.HardwareProviderImpl;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.mpc.DriveRecord;
import com.kuriosityrobotics.centerstage.mpc.MPCNode;
import com.kuriosityrobotics.centerstage.teleop.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MPC Stationary Test", group="Test")
//@Disabled
public class MPCStationaryTest extends LinearOpMode {

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

			waitForStart();

			mpcFollower.followPath(DriveRecord.zero());
//			mpcFollower.followPath(new DriveRecord.Builder(Pose.zero()).goTo(new Pose(0, 0, Math.toRadians(-90))).build());

			while (opModeIsActive())
				idle();

		} finally {
			ses.shutdownNow();
		}
	}
}