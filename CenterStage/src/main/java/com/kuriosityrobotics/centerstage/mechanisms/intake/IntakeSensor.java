package com.kuriosityrobotics.centerstage.mechanisms.intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSensor {
	private final AnalogInput sensor;

	public IntakeSensor(AnalogInput sensor) {
		this.sensor = sensor;
//		new Thread(() -> { while (!Thread.interrupted()) numPixels(); }).start(); // horrendous code, dont use
	}

	/**
	 * Returns the approximate distance the sensor measured, in meters
	 */
	private double getDistance() {
		double voltage = sensor.getVoltage();
		System.out.println("Voltage: " + voltage);
		return 0.3139821 * Math.exp(-2.59267 * voltage) + 0.0207634332;
	}

	/**
	 * Returns the number of pixels in the intake mechanism
	 */
	public int numPixels() {
		double measurement = getDistance();
		System.out.println("Measurement: " + measurement);
		return 0;
	}
}
