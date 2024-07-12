package com.kuriosityrobotics.centerstage.hardware;

public interface MetricPositionSensor {
	double getPositionMeters() throws InterruptedException;
	void updateOffsetToMatch(double currentPosition) throws InterruptedException;
}