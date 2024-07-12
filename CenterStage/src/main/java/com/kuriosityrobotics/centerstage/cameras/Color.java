package com.kuriosityrobotics.centerstage.cameras;

import org.opencv.core.Scalar;

enum Color {
	RED(
		new Scalar(100, 40, 0),
		new Scalar(140, 255, 255)
	),
	BLUE(
		new Scalar(0, 40, 0),
		new Scalar(40, 255, 255)
	);
	private final Scalar lowerBound, upperBound;

	Color(Scalar lowerBound, Scalar upperBound) {
		this.lowerBound = lowerBound;
		this.upperBound = upperBound;
	}

	public Scalar lowerBound() { return lowerBound; }
	public Scalar upperBound() { return upperBound; }
}
