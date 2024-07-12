package com.kuriosityrobotics.centerstage.cameras;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TeamElementProcessor implements VisionProcessor {
	private final Logger logger = LoggerFactory.getLogger(TeamElementProcessor.class);
	private final GameElementAnalyzer analyzer;
	private SpikeLocation spike;

	public TeamElementProcessor(GameElementAnalyzer analyzer) {
		this.analyzer = analyzer;
	}

	public synchronized SpikeLocation awaitMeasurement() throws InterruptedException {
		while(spike == null) wait();

		return spike;
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {

	}

	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		SpikeLocation spike = analyzer.estimateFor(frame);
		synchronized (this) {
			this.spike = spike;
			logger.info("AUTO (dbg): spot currently detected is " + spike);
			notifyAll();
			return spike;
		}
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

	}
}