package com.kuriosityrobotics.centerstage.cameras;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary;

import android.graphics.Canvas;

import com.kuriosityrobotics.centerstage.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

public class AprilTagOdometryProcessor implements VisionProcessor {
	private final AprilTagProcessor processor = new AprilTagProcessor.Builder()
		.setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
		.setLensIntrinsics(622.001, 622.001, 319.803, 241.251) // logitech c920
		.setTagLibrary(getCenterStageTagLibrary())
		.build();

	private final AprilTagOdometryAnalyzer analyzer;
	private final AtomicBoolean isStale = new AtomicBoolean(true);

	public AprilTagOdometryProcessor(AprilTagOdometryAnalyzer anal){
		this.analyzer = anal;
	}

	public Optional<Pose> getPoseEstimate() {
		if (!isStale.compareAndSet(false, true)) return Optional.empty();
		var detections = processor.getFreshDetections();
		if (detections == null || detections.isEmpty()) return Optional.empty();

		int count = 0;
		double x = 0;
		double y = 0;
		double theta = 0;

		for (AprilTagDetection detection : detections) {
			Pose estimate = analyzer.poseFor(detection).orElse(null);
			if (estimate != null) {
				count++;
				x += estimate.x();
				y += estimate.y();
				theta += estimate.orientation();
			}
		}

		if (count == 0) return Optional.empty();

		return Optional.of(new Pose(x / count, y / count, theta / count));
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		processor.init(width, height, calibration);
	}

	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		try {
			return processor.processFrame(frame, captureTimeNanos);
		} finally {
			isStale.set(false);
		}
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
		processor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
	}
}
