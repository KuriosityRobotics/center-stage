package com.kuriosityrobotics.centerstage.cameras;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public enum GameElementAnalyzer {
	BLUE_BOARD(
		new CameraSubmatArea(0.60, 0.80, 0.60, 0.75),
		new CameraSubmatArea(0.65, 0.85, 0.00, 0.23),
		true,
		Color.BLUE
	),
	BLUE_FIELD(
		new CameraSubmatArea(0.60, 0.83, 0.25, 0.40),
		new CameraSubmatArea(0.60, 0.95, 0.70, 1.00),
		false,
		Color.BLUE
	),
	RED_BOARD(
		new CameraSubmatArea(0.60, 0.83, 0.25, 0.40),
		new CameraSubmatArea(0.60, 0.95, 0.70, 1.00),
		false,
		Color.RED
	),
	RED_FIELD(
		new CameraSubmatArea(0.60, 0.80, 0.60, 0.75),
		new CameraSubmatArea(0.65, 0.85, 0.00, 0.23),
		true,
		Color.RED
	);

	GameElementAnalyzer(CameraSubmatArea centerRect, CameraSubmatArea sideRect, boolean seesLeft, Color color) {
		this.centerRect = centerRect;
		this.sideRect = sideRect;
		this.seesLeftSide = seesLeft;
		this.color = color;
	}
	private final CameraSubmatArea centerRect, sideRect;
	private final boolean seesLeftSide;
	private final Color color;

	private static final double THRESHOLD = 80;

	public SpikeLocation estimateFor(Mat frame) {
		Mat temp = new Mat();
		Imgproc.cvtColor(frame, temp, Imgproc.COLOR_BGR2HSV);

		Core.inRange(temp, color.lowerBound(), color.upperBound(), temp);

		double centerAverage = Core.sumElems(centerRect.extract(temp)).val[0] / centerRect.areaWith(temp);
		double sideAverage = Core.sumElems(sideRect.extract(temp)).val[0] / sideRect.areaWith(temp);
		System.out.println("Center: " + centerAverage + "  \tSide: " + sideAverage);

//		Imgproc.rectangle(frame, centerRect.rectangleFor(frame), new Scalar(255, 0, 0), 5);
//		Imgproc.rectangle(frame, sideRect.rectangleFor(frame), new Scalar(255, 0, 0), 5);
		temp.copyTo(frame);
		Imgproc.rectangle(frame, centerRect.rectangleFor(frame), new Scalar(128, 0, 0), 5);
		Imgproc.rectangle(frame, sideRect.rectangleFor(frame), new Scalar(128, 0, 0), 5);

		if (centerAverage > THRESHOLD) {
			return SpikeLocation.CENTER;
		} else if (sideAverage > THRESHOLD) {
			return seesLeftSide ? SpikeLocation.LEFT : SpikeLocation.RIGHT;
		} else {
			return seesLeftSide ? SpikeLocation.RIGHT : SpikeLocation.LEFT;
		}
	}
}