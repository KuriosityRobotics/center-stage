package com.kuriosityrobotics.centerstage.mechanisms;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DrivetrainNode {
	private final DcMotorEx frontLeft;
	private final DcMotorEx frontRight;
	private final DcMotorEx backLeft;
	private final DcMotorEx backRight;

	public DrivetrainNode(
		DcMotorEx frontLeft,
		DcMotorEx frontRight,
		DcMotorEx backLeft,
		DcMotorEx backRight
	) {
		frontLeft.setCurrentAlert(50, CurrentUnit.AMPS);
		frontRight.setCurrentAlert(50, CurrentUnit.AMPS);
		backLeft.setCurrentAlert(50, CurrentUnit.AMPS);
		backRight.setCurrentAlert(50, CurrentUnit.AMPS);

		this.frontLeft = frontLeft;
		this.frontRight = frontRight;
		this.backLeft = backLeft;
		this.backRight = backRight;

		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void setMotorVoltages(MotorPowers powers) {
		if (frontLeft.isOverCurrent() || frontRight.isOverCurrent() || backLeft.isOverCurrent() || backRight.isOverCurrent()) {
//			powers = MotorPowers.ofPowers(0, 0, 0, 0);
			System.out.println("Over current: " + powers);
		}

		frontLeft.setPower(powers.powerFrontLeft());
		backLeft.setPower(powers.powerBackLeft());
		frontRight.setPower(powers.powerFrontRight());
		backRight.setPower(powers.powerBackRight());
	}

	public void setBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
		frontLeft.setZeroPowerBehavior(brakeMode);
		frontRight.setZeroPowerBehavior(brakeMode);
		backLeft.setZeroPowerBehavior(brakeMode);
		backRight.setZeroPowerBehavior(brakeMode);
	}

	public MotorPowers getMotorPowers() {
		return MotorPowers.ofPowers(
			frontLeft.getPower(),
			frontRight.getPower(),
			backLeft.getPower(),
			backRight.getPower()
		);
	}

	public void testRoutine(Tester tester) throws InterruptedException {
		tester.header("[Drivetrain]");

		setMotorVoltages(MotorPowers.ofPowers(0.2, 0.2, 0.2, 0.2));
		tester.confirmThat("Driving Forwards [++++]?");

		setMotorVoltages(MotorPowers.ofPowers(-0.2, 0.2, 0.2, -0.2));
		tester.confirmThat("Driving Leftwards? [-++-]");

		setMotorVoltages(MotorPowers.ofPowers(-0.2, 0.2, -0.2, 0.2));
		tester.confirmThat("Turning Leftwards? [-+-+]");

		setMotorVoltages(MotorPowers.zero());
		tester.confirmThat("Is Stationary? [0000]");
	}
}
