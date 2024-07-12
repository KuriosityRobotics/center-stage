package com.kuriosityrobotics.centerstage.mpc;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class StableVoltageSensor {

	private static final int NUM_MEASUREMENTS = 50;
	private final double[] measurements = new double[NUM_MEASUREMENTS];
	private double total;
	private int index = 0;
	private final VoltageSensor sensor;

	public StableVoltageSensor(ScheduledExecutorService ses, VoltageSensor sensor) {
		this.sensor = sensor;

		double voltage = sensor.getVoltage();

		Arrays.fill(measurements, voltage);
		total = voltage * NUM_MEASUREMENTS;

		ses.scheduleAtFixedRate(this::update, 0, 1000/50, TimeUnit.MILLISECONDS);
	}

	private void update() {
		addMeasurement(sensor.getVoltage());
	}

	private synchronized void addMeasurement(double measurement) {
		total -= measurements[index];
		measurements[index] = measurement;
		total += measurements[index];
		index = (index + 1) % NUM_MEASUREMENTS;
	}

	public synchronized double getVoltage() {
		return total / NUM_MEASUREMENTS;
	}
}
