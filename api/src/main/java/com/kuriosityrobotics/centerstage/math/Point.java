package com.kuriosityrobotics.centerstage.math;

import java.io.Serializable;

public class Point implements Serializable {
	private final double x, y;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public double distance(Point other) {
		return Math.hypot(other.x() - x(), other.y() - y());
	}

	public boolean equals(Object o) {
		if (!(o instanceof Point)) {
			return false;
		}

		Point point = (Point) o;
		return MathUtil.doublesEqual(point.x(), x()) && MathUtil.doublesEqual(point.y(), y());
	}

	public Point scalarMutliply(double scalar) {
		return new Point(x() * scalar, y() * scalar);
	}

	public double x() {
		return x;
	}

	public double y() {
		return y;
	}

	public Point subtract(Point other) {
		return new Point(this.x() - other.x(), this.y() - other.y());
	}

	public Point add(Point other) {
		return new Point(this.x() + other.x(), this.y() + other.y());
	}

	public static Point zero() {
		return new Point(0, 0);
	}

	public Point rotate(double angle) {
		return new Point(
			x() * Math.cos(angle) - y() * Math.sin(angle),
			x() * Math.sin(angle) + y() * Math.cos(angle)
		);
	}

	@Override
	public String toString() {
		return "x: " + x() + " y: " + y();
	}
}
