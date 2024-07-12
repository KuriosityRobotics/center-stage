package com.kuriosityrobotics.centerstage.math;

import com.kuriosityrobotics.centerstage.util.CSVParser;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class Path {
	private final List<Point> points;

	private final List<Segment> segments;

	private final double[] cumulativeSegmentLength;

	public Path(List<Point> points) {
		this.points = List.copyOf(points);

		var segments = new ArrayList<Segment>(this.points.size() - 1);

		for (int i = 0; i < this.points.size() - 1; i++) {
			segments.add(new Segment(this.points.get(i), this.points.get(i + 1)));
		}

		this.segments = Collections.unmodifiableList(segments);

		cumulativeSegmentLength = new double[segments.size() + 1];

		for (int i = 0; i < segments.size(); i++) {
			cumulativeSegmentLength[i + 1] = cumulativeSegmentLength[i] + segments.get(i).length();
		}
	}

	public Path(Point... points) {
		this(List.of(points));
	}

	public Point get(int i) {
		return points.get(i);
	}

	private Segment getSegment(int i) {
		return segments.get(i);
	}

	public int size() {
		return points.size();
	}

	public static Path fromCSV(InputStream reader) throws IOException {
		var csv = CSVParser.parse(reader);
		var points = csv.stream().map(
			record -> new Point(
				Double.parseDouble(record.get("x_desired")),
				Double.parseDouble(record.get("y_desired"))
			)
		).collect(Collectors.toList());

		return new Path(points);
	}

	public double pathLength() {
		return partialPathLength(segments.size());
	}

	private double partialPathLength(int segments) {
		return cumulativeSegmentLength[segments];
	}

	private int getSegmentIndex(double length) {
		if (length <= 0) return 0;
		if (length >= pathLength()) return segments.size() - 1;

		return findHighestIndexNoMoreThan(cumulativeSegmentLength, length);
	}

	// creates a new path from the startRange to the endRange
	private Path extractRange(double startRange, double endRange) {
		startRange = Math.max(0, startRange);
		endRange = Math.min(pathLength(), endRange);

		int startIndex = getSegmentIndex(startRange);
		int endIndex = getSegmentIndex(endRange);

		var extractedPoints = new ArrayList<Point>();

		extractedPoints.add(distanceAlong(startRange));

		// add all the waypoints of the path between start and end distance
		for (int i = startIndex; i < endIndex; i++) {
			extractedPoints.add(getSegment(i).getEnd());
		}

		if (!(getSegment(endIndex).getStart().equals(distanceAlong(endRange)))) extractedPoints.add(distanceAlong(endRange));

		return new Path(extractedPoints);
	}

	public static int findHighestIndexNoMoreThan(double[] arr, double value) {
		int low = 0;
		int high = arr.length - 1;
		int result = -1;

		while (low <= high) {
			int mid = low + (high - low) / 2;

			if (arr[mid] == value)
				return mid;

			if (arr[mid] < value) {
				result = mid;
				low = mid + 1;
			} else {
				high = mid - 1;
			}
		}

		return result;
	}

	public Point distanceAlong(double distance) {
		if (distance <= 0) return points.get(0);
		if (distance >= pathLength()) return points.get(points.size() - 1);
		int fullSegments = findHighestIndexNoMoreThan(cumulativeSegmentLength, distance);
		return segments.get(fullSegments).distanceAlong(distance - partialPathLength(fullSegments));
	}

	// returns the distance traversed to the closest point on the path
	public double nearestDistanceAlong(Point point) {
		double nearestLength = 0;
		double nearestDistance = point.distance(distanceAlong(nearestLength));

		for (int i=0; i<segments.size(); i++) {
			double length = partialPathLength(i) + getSegment(i).nearestDistanceAlongInSegment(point);
			double distance = point.distance(distanceAlong(length));

			if (distance < nearestDistance) { // if distance is equal, prefers the point earlier in path (unless u swap with <=)
				nearestLength = length;
				nearestDistance = distance;
			}
		}

		return nearestLength;
	}

	public double nearestDistanceAlongInRange(Point point, double minRange, double maxRange) {
		minRange = Math.max(0, minRange);
		maxRange = Math.min(pathLength(), maxRange);
		return minRange + extractRange(minRange, maxRange).nearestDistanceAlong(point);
	}

	private static final double INTERPOLATION_STEP = 0.1; // for curvature only; big step size

	public double curvature(double length) {
		double stepBack = INTERPOLATION_STEP;
		double stepForward = INTERPOLATION_STEP;

		if (length == 0 || length == pathLength()) {
			return 0;
		}

		if (length - INTERPOLATION_STEP < 0) {
			stepBack = length;
		}
		if (length + INTERPOLATION_STEP > pathLength()) {
			stepForward = pathLength() - length;
		}

		// curvature = sqrt(y''(l)^2 + x''(l)^2)
		Point secondDerivatives =
			distanceAlong(length + INTERPOLATION_STEP)
			.add(distanceAlong(length - INTERPOLATION_STEP))
			.add(distanceAlong(length).scalarMutliply(-2.))
			.scalarMutliply(1. / (stepBack * stepForward));

		double curvature = secondDerivatives.distance(Point.zero()); // silly hack to get magnitude of the secondderivatives vector
		if (curvature > 5) {
			return 5;
		}

		return curvature;
	}

	// generates a backwards path
	public Path inverse() {
		var points = new ArrayList<>(this.points);
		Collections.reverse(points);
		return new Path(points);
	}

	public Path fromStartingPoint(Point point) {
		// returns this path but with the starting point at the beginning of the path
		var points = new ArrayList<Point>(size()+1);
		points.add(point);
		points.addAll(this.points);
		return new Path(points);
	}

	// for JNI
	// returns [x1, y1, x2, y2, ...]
	private double[] toDoubleArray() {
		double[] result = new double[size() * 2];
		for (int i=0; i<size(); i++) {
			result[i*2] = get(i).x();
			result[i*2+1] = get(i).y();
		}
		return result;
	}
}
