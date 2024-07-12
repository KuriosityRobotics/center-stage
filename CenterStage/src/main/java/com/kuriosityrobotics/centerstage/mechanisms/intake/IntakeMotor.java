package com.kuriosityrobotics.centerstage.mechanisms.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

public class IntakeMotor {
	private final DcMotorEx delegate;

	public IntakeMotor(DcMotorEx delegate) {
		this.delegate = delegate;

		delegate.setMode(RUN_WITHOUT_ENCODER);
		delegate.setZeroPowerBehavior(FLOAT);
		delegate.setPower(0);
		delegate.setCurrentAlert(20, AMPS);
	}

	public void setPower(double power) {
		delegate.setPower(power);
	}

	public void setPower(IntakeSpeed speed) {
		setPower(speed.power);
	}
}