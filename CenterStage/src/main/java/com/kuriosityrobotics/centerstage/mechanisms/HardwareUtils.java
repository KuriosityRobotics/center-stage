package com.kuriosityrobotics.centerstage.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class HardwareUtils {
	public static MotorState saveMotorState(DcMotorEx motor) {
		return new MotorState(motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION),
				motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER), motor.getPower(), motor.getMode());
	}

	public static void restoreMotorState(DcMotorEx motor, MotorState state) {
		motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, state.runToPositionPIDF);
		motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, state.runUsingEncoderPIDF);
		motor.setPower(state.power);
		motor.setMode(state.runMode);
	}

	public static class MotorState {
		public final PIDFCoefficients runToPositionPIDF, runUsingEncoderPIDF;
		public final double power;
		public final DcMotor.RunMode runMode;

        public MotorState(PIDFCoefficients runToPositionPIDF, PIDFCoefficients runUsingEncoderPIDF, double power, DcMotor.RunMode runMode) {
            this.runToPositionPIDF = runToPositionPIDF;
            this.runUsingEncoderPIDF = runUsingEncoderPIDF;
            this.power = power;
            this.runMode = runMode;
        }
    }
}
