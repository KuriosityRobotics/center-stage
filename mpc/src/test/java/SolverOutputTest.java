import com.kuriosityrobotics.centerstage.localisation.messages.LocalisationDatum;
import com.kuriosityrobotics.centerstage.mpc.*;
import com.kuriosityrobotics.centerstage.util.Instant;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import static com.kuriosityrobotics.centerstage.mpc.SolverOutput.NUM_STAGES;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;

class SolverOutputTest {

	/* Constantly runs the solver for ease of Profiling */
	@Disabled
	@Test
	void testSolverProcess() {
		for (int i = 0; i < 100000; i++) {
			driveForwards();
		}
	}

	@Test
	@Disabled
	void testStationary() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				0, 0, 0,
				0, 0, 0,
				1
			));

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					0, 0, 0, 0,
					0, 0, 0,
					0, 0, 0
				))
				.setTargetsFor(i, new DriveTargets(
					0, 0, 0,
					0, 0, 0
				));
		}

		var solution = builder.build(12).solve();

		assertEquals(1, solution.getExitCode());
		for (var state : solution.getStates()) {
			assertEquals( 0, state.getFl(), 0.001);
			assertEquals( 0, state.getFr(), 0.001);
			assertEquals( 0, state.getBl(), 0.001);
			assertEquals( 0, state.getBr(), 0.001);
		}

		assertEquals(0, solution.getSolutionInfo().getPobj(), 0.001);
	}

	@Test
	@Disabled
	void testObjective() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				1, 1, 1,
				0, 0, 0,
				10000
			))
			.startingAt(LocalisationDatum.zero());

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					0, 0, 0, 0,
					0, 0, 0,
					0, 0, 0
				))
				.setTargetsFor(i, new DriveTargets(
					1, 1, 1,
					0, 0, 0
				));
		}

		var solution = builder.build(12).solve();

		assertEquals(1, solution.getExitCode());
		for (var state : solution.getStates()) {
			assertEquals( 0, state.getFl(), 0.001);
			assertEquals( 0, state.getFr(), 0.001);
			assertEquals( 0, state.getBl(), 0.001);
			assertEquals( 0, state.getBr(), 0.001);
		}

		assertEquals(1.5 * solution.getStates().length, solution.getSolutionInfo().getPobj(), 0.1);
	}

	@Test
	@Disabled
	void testSolver() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				1, 1, 1,
				1, 1, 1,
				0.01
			));

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					0, 0, 0, 0,
					0, 0, 0,
					0, 0, 0
				))
				.setTargetsFor(i, new DriveTargets(
					0, 0, 0,
					0, 0, 0
				));
		}

		var input = builder.build(12);
		var output = input.solve();

		System.out.println(output);

		assertEquals(1, output.getExitCode());
	}

	private static final double FORWARDS_SPEED = 0.2; // m/s

	@Test
	@Disabled
	void driveForwards() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				1, 1, 1,
				1, 1, 1,
				0.01
			));

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					+0.5, +0.5, +0.5, +0.5,
					0, 0, 0,
					0, 0, 0
				))
				.setTargetsFor(i, new DriveTargets(
					i * 0.1 * FORWARDS_SPEED, 0, 0,
					FORWARDS_SPEED, 0, 0
				));
		}

		var input = builder.build(12);

		Instant start = Instant.now();
		var output = input.solve();
		Instant end = Instant.now();

		System.out.println("Time taken: " + end.since(start).toNanos() / 1000 + " microseconds");
		System.out.println(output);

		assertEquals(1, output.getExitCode());
	}

	private static final double SIDEWAYS_SPEED = 0.2; // m/s

	@Test
	@Disabled
	void driveSideways() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				1, 1, 1,
				1, 1, 1,
				0.01
			));

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					-0.5, +0.5, +0.5, -0.5,
					0, 0, 0,
					0, 0, 0
				))
				.setTargetsFor(i, new DriveTargets(
					0, i * 0.1 * SIDEWAYS_SPEED, 0,
					0, SIDEWAYS_SPEED, 0
				));
		}

		var input = builder.build(12);

		Instant start = Instant.now();
		var output = input.solve();
		Instant end = Instant.now();

		System.out.println("Time taken: " + end.since(start).toNanos() / 1000 + " microseconds");
		System.out.println(output);

		assertEquals(1, output.getExitCode());
	}

	@Test
	@Disabled
	void driveDiagonally() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				1, 1, 1,
				1, 1, 1,
				0.01
			));

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					0, +0.5, +0.5, 0,
					0, 0, 0,
					0, 0, 0
				))
				.setTargetsFor(i, new DriveTargets(
					i * 0.1 * FORWARDS_SPEED, i * 0.1 * SIDEWAYS_SPEED, 0,
					FORWARDS_SPEED, SIDEWAYS_SPEED, 0
				));
		}

		var input = builder.build(12);

		Instant start = Instant.now();
		var output = input.solve();
		Instant end = Instant.now();

		System.out.println("Time taken: " + end.since(start).toNanos() / 1000 + " microseconds");
		System.out.println(output);

		assertEquals(1, output.getExitCode());
	}

	private static final double ROTATION_SPEED = 0.5; // rad/s

	@Test
	@Disabled
	void driveSpin() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				1, 1, 1,
				0.5, 0.5, 0.5,
				0.001
			));

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					-0.2, +0.2, -0.2, +0.2,
					0, 0, 0.1 * i * ROTATION_SPEED,
					0, 0, ROTATION_SPEED
				))
				.setTargetsFor(i, new DriveTargets(
					0, 0, 0.1 * i * ROTATION_SPEED,
					0, 0, ROTATION_SPEED
				));
		}

		var input = builder.build(12);

		Instant start = Instant.now();
		var output = input.solve();
		Instant end = Instant.now();

		System.out.println("Time taken: " + end.since(start).toNanos() / 1000 + " microseconds");
		System.out.println(output);

		assertEquals(1, output.getExitCode());
	}

	@Test
	@Disabled
	void driveForwardsSpinning() {
		var builder = new SolverInputBuilder()
			.withDefaultParameters()
			.withWeights(new DriveWeights(
				1, 1, 1,
				1, 1, 1,
				0.01
			));

		for (int i = 0; i < NUM_STAGES; i++) {
			builder
				.setInitialGuessFor(i, SystemState.from(
					0, +0.5, 0, +0.5,
					0, 0, 0,
					0, 0, 0
				))
				.setTargetsFor(i, new DriveTargets(
					0.1 * i * FORWARDS_SPEED, 0, 0.1 * i * ROTATION_SPEED,
					FORWARDS_SPEED, 0, ROTATION_SPEED
				));
		}

		var input = builder.build(12);

		Instant start = Instant.now();
		var output = input.solve();
		Instant end = Instant.now();

		System.out.println("Time taken: " + end.since(start).toNanos() / 1000 + " microseconds");
		System.out.println(output);

		assertEquals(1, output.getExitCode());
	}

	@Test
	@Disabled
	public void testLoadRecord() throws IOException {
		try (var stream = getClass().getClassLoader().getResourceAsStream("forwards_back.csv")) {
			var record = DriveRecord.fromCSV(stream);
			var builder = new SolverInputBuilder()
				.withDefaultParameters()
				.withWeights(new DriveWeights(
					1000, 1000, 1000,
					0, 0, 0,
					0.001
				));

			double timeAlongPath = 0;
			SystemState currentState = record.get(timeAlongPath);

			// on the first solve, initialize guesses using record
			for (int i = 0; i < NUM_STAGES; i++) {
				SystemState position = record.get(timeAlongPath + i * 0.1);
				builder.setInitialGuessFor(i, position);
			}

			for (int step = 0; step < record.maxTime() * 20; step++) {
				for (int i = 0; i < NUM_STAGES; i++) {
					SystemState position = record.get(timeAlongPath + i * 0.1);
					builder.setTargetsFor(i, DriveTargets.fromSystemState(position));
				}

				builder.startingAt(currentState);

				var input = builder.build(12);

				Instant start = Instant.now();
				var output = input.solve();
				Instant end = Instant.now();

				System.out.println("Time taken: " + end.since(start).toNanos() / 1000 + " microseconds");
				System.out.println(output);

				currentState = output.getStates()[1]; // assume perfect physics

				timeAlongPath += 0.1;

				var targetState = record.get(timeAlongPath);
				double positionError = currentState.linearDistanceTo(record.get(timeAlongPath));

				System.out.println();
				System.out.println("Step: " + step);
				System.out.println("Time along path: " + timeAlongPath);
				System.out.println("Target: " + targetState);
				System.out.println("Actual: " + currentState);
				System.out.println("Error: " + positionError + " metres");

				if (positionError > 0.05)
					timeAlongPath = record.nearestInterpolatedTimeInRange(currentState, timeAlongPath - 0.1, timeAlongPath + 0.1, 0.01);

				timeAlongPath = Math.min(timeAlongPath, record.maxTime());

				System.out.println("New time along path: " + timeAlongPath);
				System.out.println();

				builder.withPreviousSolution(output); // prepare next solve
			}
		}
	}

	@Test
	@Disabled
	public void testLoadSimpleRecord() throws IOException {
		try (var stream = getClass().getClassLoader().getResourceAsStream("programmed_drive_spin.csv")) {
			var record = DriveRecord.fromCSV(stream);
			var builder = new SolverInputBuilder()
				.withDefaultParameters()
				.withWeights(new DriveWeights(
					1000, 1000, 1000,
					10, 10, 10,
					0.001
				));

			double timeAlongPath = 0;
			SystemState currentState = record.get(timeAlongPath);

			// on the first solve, initialize guesses using record
			for (int i = 0; i < NUM_STAGES; i++) {
				SystemState position = record.get(timeAlongPath + i * 0.1);
				builder.setInitialGuessFor(i, position);
			}

			for (int step = 0; step < record.maxTime() * 20; step++) {
				for (int i = 0; i < NUM_STAGES; i++) {
					SystemState position = record.get(timeAlongPath + i * 0.1);
					builder.setTargetsFor(i, DriveTargets.fromSystemState(position));
				}

				builder.startingAt(currentState);

				var input = builder.build(12);

				Instant start = Instant.now();
				var output = input.solve();
				Instant end = Instant.now();

				System.out.println("Time taken: " + end.since(start).toNanos() / 1000 + " microseconds");
				System.out.println(output);

				currentState = output.getStates()[1]; // assume perfect physics

				timeAlongPath += 0.1;

				var targetState = record.get(timeAlongPath);
				double positionError = currentState.linearDistanceTo(record.get(timeAlongPath));

				System.out.println();
				System.out.println("Step: " + step);
				System.out.println("Time along path: " + timeAlongPath);
				System.out.println("Target: " + targetState);
				System.out.println("Actual: " + currentState);
				System.out.println("Error: " + positionError + " metres");

				if (positionError > 0.05)
					timeAlongPath = record.nearestInterpolatedTimeInRange(currentState, timeAlongPath - 0.1, timeAlongPath + 0.1, 0.01);

				timeAlongPath = Math.min(timeAlongPath, record.maxTime());

				System.out.println("New time along path: " + timeAlongPath);
				System.out.println();

				builder.withPreviousSolution(output); // prepare next solve
			}
		}
	}
}
