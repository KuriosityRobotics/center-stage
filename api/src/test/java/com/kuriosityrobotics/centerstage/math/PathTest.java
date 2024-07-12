package com.kuriosityrobotics.centerstage.math;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

public class PathTest {
	@Test
	public void testPathConstruction() {
		Point[] wayPoints = new Point[] {
			new Point(0, 0),
			new Point(1, 0),
			new Point(1, 1),
			new Point(2, 0)
		};

		Path path = new Path(wayPoints);
	}

	@Test
	public void testPathLength() {
		Point[] wayPoints = new Point[] {
			new Point(0, 0),
			new Point(1, 0),
			new Point(1, 1),
			new Point(2, 0)
		};

		Path path = new Path(wayPoints);

		assertEquals((2 + Math.sqrt(2)), path.pathLength());
	}

	@Test
	public void testDistanceAlong() {
		Point[] wayPoints = new Point[] {
			new Point(0, 0),
			new Point(1, 0),
			new Point(1, 1),
			new Point(2, 0)
		};

		Path path = new Path(wayPoints);

		assertEquals(new Point(0,0), path.distanceAlong(-5)); // before the start; returns the start position
		assertEquals(new Point(0.5,0), path.distanceAlong(0.5));
		assertEquals(new Point(0,0), path.distanceAlong(0));
		assertEquals(new Point(1,0), path.distanceAlong(1));
		assertEquals(new Point(1,0.5), path.distanceAlong(1.5));
		assertEquals(new Point(1,1), path.distanceAlong(2));
		assertEquals(new Point(1.5,0.5), path.distanceAlong(2 + Math.sqrt(2) / 2));
		assertEquals(new Point(2,0), path.distanceAlong(2 + Math.sqrt(2)));
		assertEquals(new Point(2,0), path.distanceAlong(2 + Math.sqrt(2) + 5)); // after the end; returns the end position
	}

	@Test
	public void testNearestDistanceAlong() {
		Point[] wayPoints = new Point[] {
			new Point(0, 0),
			new Point(1, 0),
			new Point(1, 1),
			new Point(2, 0)
		};

		Path path = new Path(wayPoints);

		assertEquals(0, path.nearestDistanceAlong(new Point(-1,-1)));
		assertEquals(0, path.nearestDistanceAlong(new Point(0,0)));
		assertEquals(0.5, path.nearestDistanceAlong(new Point(0.5,-1)));
		assertEquals(0.5, path.nearestDistanceAlong(new Point(0.5,0.5)));
		assertEquals(1, path.nearestDistanceAlong(new Point(1.1,-0.1)));
		assertEquals(1, path.nearestDistanceAlong(new Point(1.5,-20))); // priortises start of path
		assertEquals(2, path.nearestDistanceAlong(new Point(1.1,1.1)));
		assertEquals(2, path.nearestDistanceAlong(new Point(0,20)));
		assertEquals(2 + Math.sqrt(2) / 2, path.nearestDistanceAlong(new Point(1.5,0.5)));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlong(new Point(2,0)));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlong(new Point(1.501,-20)));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlong(new Point(3,-20)));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlong(new Point(20, 0)));
	}

	@Test
	public void testNearestDistanceAlongInRange() {
		Point[] wayPoints = new Point[] {
			new Point(0, 0),
			new Point(1, 0),
			new Point(1, 1),
			new Point(2, 0)
		};

		Path path = new Path(wayPoints);

		assertEquals(0, path.nearestDistanceAlongInRange(new Point(-1,-1),0, 1));
		assertEquals(0, path.nearestDistanceAlongInRange(new Point(-1,-1),-5, 5));
		assertEquals(0.5, path.nearestDistanceAlongInRange(new Point(-1,-1),0.5, 1));
		assertEquals(0.5, path.nearestDistanceAlongInRange(new Point(0.5,-1), 0, 1));
		assertEquals(0.5, path.nearestDistanceAlongInRange(new Point(0.5,0.5), 0, 1));
		assertEquals(0.5, path.nearestDistanceAlongInRange(new Point(0,0.5), 0.5, 1));
		assertEquals(1, path.nearestDistanceAlongInRange(new Point(1.1,-0.1), 0, 2));
		assertEquals(1, path.nearestDistanceAlongInRange(new Point(2,0), 0, 1));
		assertEquals(1, path.nearestDistanceAlongInRange(new Point(1.5,-20), 0, 2 + Math.sqrt(2))); // priortises start of path
		assertEquals(1, path.nearestDistanceAlongInRange(new Point(-20,0), 1, 2 + Math.sqrt(2))); // priortises start of path
		assertEquals(1, path.nearestDistanceAlongInRange(new Point(-20,-20), 1, 2 + Math.sqrt(2))); // priortises start of path
		assertEquals(2, path.nearestDistanceAlongInRange(new Point(-20,0), 2, 2 + Math.sqrt(2))); // priortises start of path
		assertEquals(2, path.nearestDistanceAlongInRange(new Point(1.1,1.1), 0, 2));
		assertEquals(2, path.nearestDistanceAlongInRange(new Point(1.1,1.1), 2, 2 + Math.sqrt(2)));
		assertEquals(2, path.nearestDistanceAlongInRange(new Point(0,20), 1, 2 + Math.sqrt(2)));
		assertEquals(2 + Math.sqrt(2) / 2, path.nearestDistanceAlongInRange(new Point(1.5,0.5), 2, 2 + Math.sqrt(2)));
		assertEquals(2 + Math.sqrt(2) / 2, path.nearestDistanceAlongInRange(new Point(2,-20), 1.5, 2 + Math.sqrt(2) / 2));
		assertEquals(2 + Math.sqrt(2) / 2, path.nearestDistanceAlongInRange(new Point(20, 0), 0, 2 + Math.sqrt(2) / 2));
		assertEquals(3, path.nearestDistanceAlongInRange(new Point(2,-20), 2, 3));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlongInRange(new Point(2,0), 0, 2 + Math.sqrt(2)));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlongInRange(new Point(1.5,-20),1.01, 2 + Math.sqrt(2)));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlongInRange(new Point(1.501,-20),1, 2 + Math.sqrt(2)));
		assertEquals(2 + Math.sqrt(2), path.nearestDistanceAlongInRange(new Point(1,-1), 2, 2 + Math.sqrt(2)));
	}

	@Test
	public void testPath() {
		var path = new Path(new Point[] {
			new Point(0, 0),
			new Point(1, 0),
			new Point(1, 2),
			new Point(0.5, 2)
		});

		var interpolatedTargets = new Point[] {
			new Point(0.25,0),
			new Point(0.5,0),
			new Point(0.75,0),
			new Point(1,0),
			new Point(1,0.25),
			new Point(1,0.5),
			new Point(1,0.75),
			new Point(1,1),
			new Point(1,1.25),
			new Point(1,1.5),
			new Point(1,1.75),
			new Point(1,2),
			new Point(0.75,2),
			new Point(0.5,2)
		};

		var targetLengths = newTargetLengths(0, 0.25, interpolatedTargets.length);

		for (int i=0; i<interpolatedTargets.length; i++){
			assertEquals(new double[] {0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5}[i], targetLengths[i]);
		}

		for (int i=0; i<interpolatedTargets.length; i++){
			assertEquals(interpolatedTargets[i], path.distanceAlong(targetLengths[i]));
		}
	}

	@Test
	@Disabled
	public void testCSVLoad() throws IOException {
		try (var pathStream = getClass().getClassLoader().getResourceAsStream("path_points_list.csv")) {
			var path = Path.fromCSV(pathStream);

			assertEquals(path.distanceAlong(0).x(), 0.30804555555555974);
			assertEquals(path.distanceAlong(0).y(), 0.9007121222221852);
		}
	}

	@Test
	public void testInverse() {
		Path path = new Path(new Point[] {
			new Point(0, 0),
			new Point(1, 0),
			new Point(2, 0),
			new Point(3, 2),
			new Point(4, 2),
		});

		Path inverse = path.inverse();

		assertEquals(path.get(0), inverse.get(4));
		assertEquals(path.get(1), inverse.get(3));
		assertEquals(path.get(2), inverse.get(2));
		assertEquals(path.get(3), inverse.get(1));
		assertEquals(path.get(4), inverse.get(0));

		assertEquals(path.distanceAlong(path.pathLength()/2), inverse.distanceAlong(path.pathLength()/2));
		assertEquals(path.distanceAlong(path.pathLength()/4), inverse.distanceAlong(path.pathLength()*3/4));
		assertEquals(path.pathLength(), inverse.pathLength());
	}

	@Test
	public void testZeroPath() {
		Path path = new Path(
			Point.zero(),
			Point.zero()
		);

		assertEquals(path.distanceAlong(0), Point.zero());
		assertEquals(path.distanceAlong(1), Point.zero());

		assertEquals(path.nearestDistanceAlong(new Point(0, 0)), 0);
		assertEquals(path.nearestDistanceAlong(new Point(1, 1)), 0);
	}

	private double[] newTargetLengths(double previousTarget, double increment, int numStages) {
		var targets = new double[numStages];
		for (int i = 0; i < numStages; i++) {
			targets[i] = previousTarget + increment * (i + 1);
		}
		return targets;
	}
}
