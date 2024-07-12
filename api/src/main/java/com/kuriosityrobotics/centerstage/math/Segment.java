package com.kuriosityrobotics.centerstage.math;

public class Segment {
	private final Point start;
	private final Point end;

	public Segment(Point start, Point end) {
		this.start = start;
		this.end = end;
	}

	public Point getStart() {
		return start;
	}

	public Point getEnd() {
		return end;
	}

	public double length() {
		return start.distance(end);
	}

	public Point distanceAlong(double distance) {
		if (length() == 0) {
			return start;
		}
		return start.add(end.subtract(start).scalarMutliply(distance / length()));
	}

	public double nearestDistanceAlong(Point point) {
		if (length() == 0) {
			return 0.;
		}
		// study geometry
		return -((end.x() - start.x()) * (start.x() - point.x()) + (end.y() - start.y()) * (start.y() - point.y())) / length();
	}

	public double nearestDistanceAlongInSegment(Point point) {
		double nearestDistance = nearestDistanceAlong(point);
		return Math.max(0, Math.min(length(), nearestDistance));
	}
}
