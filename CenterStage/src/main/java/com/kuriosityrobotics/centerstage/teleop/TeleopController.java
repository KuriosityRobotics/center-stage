package com.kuriosityrobotics.centerstage.teleop;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.mechanisms.DrivetrainNode;
import com.kuriosityrobotics.centerstage.mechanisms.MechanismManager;
import com.kuriosityrobotics.centerstage.mechanisms.airplane.AirplaneNode;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeExtensionMotor;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeLiftServo;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeSpeed;
import com.kuriosityrobotics.centerstage.mechanisms.rigging.RiggingNode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import static com.kuriosityrobotics.centerstage.teleop.ButtonEdgeDetector.Button.*;
import static java.lang.Math.abs;

public class TeleopController {
	private final Gamepad gamepad1, gamepad2;
	private final DrivetrainNode drivetrainNode;

	private final MechanismManager mechanisms;

	AtomicReference<ButtonEdgeDetector.Button> intakeMode = new AtomicReference<>(null);

	public TeleopController(
		ScheduledExecutorService ses,
		Gamepad gamepad1,
		Gamepad gamepad2,
		DrivetrainNode drivetrainNode,
		MechanismManager mechanisms,
		AirplaneNode aeroplaneNode,
		RiggingNode riggingNode
	) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
		this.drivetrainNode = drivetrainNode;
		this.mechanisms = mechanisms;

		ses.scheduleWithFixedDelay(this::updateDrivetrain, 0, 1000 / 50, TimeUnit.MILLISECONDS);

		var gp1EdgeDetector = new ButtonEdgeDetector(ses, gamepad1);
		var gp2EdgeDetector = new ButtonEdgeDetector(ses, gamepad2);

		// aeroplane
		gp1EdgeDetector.onRising(Y, aeroplaneNode::launchAirplane);

		gp1EdgeDetector.onRising(A, this::flipDirection);
		gp1EdgeDetector.onRising(RIGHT_TRIGGER, this::slowDrive);
		gp1EdgeDetector.onFalling(RIGHT_TRIGGER, this::normalDrive);

		// start rigging
		gp1EdgeDetector.onRising(DPAD_UP, riggingNode::raise);
		gp1EdgeDetector.onRising(DPAD_LEFT, riggingNode::raiseLeft);
		gp1EdgeDetector.onRising(DPAD_RIGHT, riggingNode::raiseRight);
		gp1EdgeDetector.onFalling(DPAD_UP, riggingNode::hold);
		gp1EdgeDetector.onFalling(DPAD_LEFT, riggingNode::hold);
		gp1EdgeDetector.onFalling(DPAD_RIGHT, riggingNode::hold);

//		 clasp rigging
		gp1EdgeDetector.onRising(DPAD_DOWN, riggingNode::clasp);
		gp1EdgeDetector.onFalling(DPAD_DOWN, riggingNode::hold);

		// custom depositing
		gp1EdgeDetector.onRising(LEFT_BUMPER, mechanisms::releaseClaw2);
		gp1EdgeDetector.onRising(LEFT_TRIGGER, mechanisms::releaseClaw1);

		// start intake
		gp2EdgeDetector.onRising(Y, () -> {
			if (intakeMode.compareAndSet(null, Y))
				mechanisms.setIntakeSpeed(IntakeSpeed.FAST);
		});
		gp2EdgeDetector.onRising(X, () -> {
			if (intakeMode.compareAndSet(null, X))
				mechanisms.setIntakeSpeed(IntakeSpeed.REVERSE);
		});

		// stop intake
		gp2EdgeDetector.onFalling(X, () -> {
			if (intakeMode.compareAndSet(X, null))
				mechanisms.stopIntake();
		});
		gp2EdgeDetector.onFalling(Y, () -> {
			if (intakeMode.compareAndSet(Y, null))
				mechanisms.stopIntake();
		});

		gp2EdgeDetector.onRising(A, () -> mechanisms.extendIntake(IntakeExtensionMotor.IntakeSlidePosition.CLOSED));

		gp2EdgeDetector.onRising(RIGHT_TRIGGER, () -> mechanisms.extendIntake(IntakeExtensionMotor.IntakeSlidePosition.FULL));

		gp2EdgeDetector.onRising(LEFT_TRIGGER, mechanisms::transfer);
		gp2EdgeDetector.onRising(LEFT_BUMPER, mechanisms::releaseAndCollapse);

		gp2EdgeDetector.onRising(DPAD_UP, mechanisms::incrementHeight);
		gp2EdgeDetector.onRising(DPAD_DOWN, mechanisms::decrementHeight);


		gp2EdgeDetector.onRising(LEFT_STICK_UP, () -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.STACK_FOUR));
		gp2EdgeDetector.onFalling(LEFT_STICK_UP, () -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));
		gp2EdgeDetector.onRising(LEFT_STICK_DOWN, () -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.GROUND));
		gp2EdgeDetector.onFalling(LEFT_STICK_DOWN, () -> mechanisms.toIntakeHeight(IntakeLiftServo.IntakeHeight.LIFTED));

		gp2EdgeDetector.onRising(DPAD_LEFT, mechanisms::decrementWrist);
		gp2EdgeDetector.onRising(DPAD_RIGHT, mechanisms::incrementWrist);

		gp2EdgeDetector.onRising(RIGHT_STICK_BUTTON, mechanisms::recalibrate);
	}

	private MotorPowers movementsToMotorVoltages(double forwards, double sideways, double spin) {
		return MotorPowers.ofPowers(
			(forwards - sideways - spin),
			(forwards + sideways + spin),
			(forwards + sideways - spin),
			(forwards - sideways + spin));
	}
	private final AtomicReference<Double> driveDirection = new AtomicReference<>(1.);

	private void flipDirection() {
		driveDirection.getAndUpdate(a -> -a);
	}
	private void slowDrive() {
		driveDirection.getAndUpdate(a -> 0.5 * Math.signum(a));
	}

	private void normalDrive() {
		driveDirection.getAndUpdate(Math::signum);
	}

	private void updateDrivetrain() {
		double scale = driveDirection.get();
		drivetrainNode.setMotorVoltages(
			movementsToMotorVoltages(
				scale * Math.signum(-gamepad1.left_stick_y) * Math.pow(abs(gamepad1.left_stick_y), 1.),
				scale * Math.signum(-gamepad1.left_stick_x) * Math.pow(abs(gamepad1.left_stick_x), 1.),
				Math.abs(scale) * Math.signum(-gamepad1.right_stick_x) * Math.pow(abs(gamepad1.right_stick_x), 0.9))
		);
    }
}
