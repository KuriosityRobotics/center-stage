package com.kuriosityrobotics.centerstage.teleop;

import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.cameras.CameraNode;
import com.kuriosityrobotics.centerstage.hardware.HardwareProvider;
import com.kuriosityrobotics.centerstage.localisation.CorrectedIMU;
import com.kuriosityrobotics.centerstage.mechanisms.DrivetrainNode;
import com.kuriosityrobotics.centerstage.mechanisms.MechanismManager;
import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;
import com.kuriosityrobotics.centerstage.mechanisms.airplane.AirplaneNode;
import com.kuriosityrobotics.centerstage.mechanisms.airplane.AirplaneServo;
import com.kuriosityrobotics.centerstage.mechanisms.intake.*;
import com.kuriosityrobotics.centerstage.mechanisms.outtake.*;
import com.kuriosityrobotics.centerstage.mechanisms.rigging.RiggingMotor;
import com.kuriosityrobotics.centerstage.mechanisms.rigging.RiggingNode;
import com.kuriosityrobotics.centerstage.mpc.StableVoltageSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import static com.kuriosityrobotics.centerstage.hardware.LynxHub.CONTROL_HUB;
import static com.kuriosityrobotics.centerstage.hardware.LynxHub.EXPANSION_HUB;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class Robot {
	public static ScheduledExecutorService createScheduledExecutor() {
		return Executors.newScheduledThreadPool(32, Executors.defaultThreadFactory());
	}

	private static Outtake createOuttake(HardwareProvider hp, BulkDataFetcher bdf) throws InterruptedException {
		try (var scope = HardwareTaskScope.open()) {
			var slides = scope.fork(() -> new OuttakeExtensionSlides(
				new OuttakeMotor(bdf.notifier(CONTROL_HUB), hp.motor(CONTROL_HUB, 1, REVERSE)),
				new OuttakeMotor(bdf.notifier(CONTROL_HUB), hp.motor(CONTROL_HUB, 0, FORWARD))
			));

			var arm = scope.fork(() -> new OuttakeArm(
				new OuttakeArmLeveler(hp.servo(EXPANSION_HUB, 1)),
				new OuttakeArmAngler(hp.servo(EXPANSION_HUB, 0))
			));

			var wrist = scope.fork(() -> new OuttakeWrist(hp.servo(EXPANSION_HUB, 2)));

			// 1 is 4-bar wrist
			var claw1 = scope.fork(() -> new OuttakeClaw(hp.servo(CONTROL_HUB, 0)));
			var claw2 = scope.fork(() -> new OuttakeClaw(hp.servo(CONTROL_HUB, 4)));

			scope.join();

			return new Outtake(
				slides.get(),
				arm.get(),
				wrist.get(),
				claw1.get(),
				claw2.get()
			);
		}
	}

	private static Intake createIntake(HardwareProvider hp, BulkDataFetcher bdf) throws InterruptedException {
		try (var scope = HardwareTaskScope.open()) {
			var intakeMotor = scope.fork(() -> new IntakeMotor(hp.motor(CONTROL_HUB, 3, FORWARD)));
			var intakeExtensionMotor = scope.fork(() -> new IntakeExtensionMotor(bdf.notifier(CONTROL_HUB), hp.motor(CONTROL_HUB, 2, REVERSE)));
			var intakeContainer = scope.fork(() -> new IntakeContainer(
				new IntakeContainerServo.Left(hp.servo(EXPANSION_HUB, 4)),
				new IntakeContainerServo.Right(hp.servo(EXPANSION_HUB, 3)),
				new IntakeSensor(hp.analogInput(EXPANSION_HUB, 1))
			));
			var intakeLiftServo = scope.fork(() -> new IntakeLiftServo(hp.servo(CONTROL_HUB, 5)));

			scope.join();

			return new Intake(
				intakeMotor.get(),
				intakeExtensionMotor.get(),
				intakeContainer.get(),
				intakeLiftServo.get()
			);
		}
	}

	public static MechanismManager createMechanismManager(HardwareProvider hp, BulkDataFetcher bdf) throws InterruptedException {
		try (var scope = HardwareTaskScope.open()) {
			var intake = scope.fork(() -> createIntake(hp, bdf));
			var outtake = scope.fork(() -> createOuttake(hp, bdf));

			scope.join();

			return new MechanismManager(intake.get(), outtake.get());
		}
	}

	public static AirplaneNode createAirplaneNode(HardwareProvider hp) {
		return new AirplaneNode(new AirplaneServo(hp.servo(CONTROL_HUB, 3)));
	}

	public static RiggingNode createRiggingNode(HardwareProvider hp) {
		return new RiggingNode(
			new RiggingMotor.Left(hp.crServo(CONTROL_HUB, 1)),
			new RiggingMotor.Right(hp.crServo(CONTROL_HUB, 2))
		);
	}

	public static DrivetrainNode createDrivetrainNode(HardwareProvider hp){
		return new DrivetrainNode(
			hp.motor(EXPANSION_HUB, 1, FORWARD),
			hp.motor(EXPANSION_HUB, 2, REVERSE),
			hp.motor(EXPANSION_HUB, 0, FORWARD),
			hp.motor(EXPANSION_HUB, 3, REVERSE)
		);
	}

	public static CorrectedIMU createIMUNode(HardwareProvider hp) {
		return new CorrectedIMU(
			hp.byName(IMU.class, "imu")
		);
	}

	public static BulkDataFetcher createBulkDataFetcher(ScheduledExecutorService ses, HardwareProvider hp) {
		return new BulkDataFetcher(
			ses,
			hp.moduleFor(CONTROL_HUB),
			hp.moduleFor(EXPANSION_HUB)
		);
	}

	public static CameraNode createCameraNode(ScheduledExecutorService ses, HardwareProvider hp, VisionProcessor... vp) {
		return new CameraNode(
			ses,
			hp.byName(WebcamName.class, "Webcam 1"),
			vp
		);
	}

	public static StableVoltageSensor createStableVoltageSensor(ScheduledExecutorService ses, HardwareProvider hp) {
		return new StableVoltageSensor(
			ses,
			hp.voltageSensorFor(EXPANSION_HUB)
		);
	}
}
