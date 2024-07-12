package com.kuriosityrobotics.centerstage.cameras;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class CameraSubmatArea {
	private final double verticalStart, verticalEnd, horizontalStart, horizontalEnd;

	public CameraSubmatArea(double verticalStart, double verticalEnd, double horizontalStart, double horizontalEnd) {
		this.verticalStart = verticalStart;
		this.verticalEnd = verticalEnd;
		this.horizontalStart = horizontalStart;
		this.horizontalEnd = horizontalEnd;
	}

	public Rect rectangleFor(Mat image) {
		return new Rect(
			new Point(horizontalStart * image.width(), verticalStart * image.height()),
			new Point(horizontalEnd * image.width(), verticalEnd * image.height())
		);
	}

	public double areaWith(Mat image) {
		return (horizontalEnd - horizontalStart) * image.width()
			* (verticalEnd - verticalStart) * image.height();
	}

	public Mat extract(Mat source) {
		return source.submat(rectangleFor(source));
	}
}
