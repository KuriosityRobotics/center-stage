package com.kuriosityrobotics.centerstage.mpc;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.localisation.messages.LocalisationDatum;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;
import com.kuriosityrobotics.centerstage.util.CSVParser;
import com.kuriosityrobotics.centerstage.util.Duration;

import java.io.IOException;
import java.io.InputStream;
import java.util.TreeMap;

public class DriveRecord {
	private final TreeMap<Double, SystemState> states;

	private DriveRecord(TreeMap<Double, SystemState> states) {
		this.states = new TreeMap<>(states);
	}

	public static DriveRecord loadFromResource(String resourceName) {
		try (var inputStream = DriveRecord.class.getClassLoader().getResourceAsStream(resourceName);) {
			return fromCSV(inputStream);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	/**
	 * Parses this csv header:
	 * <code>
	 *     time,battery_voltage,fl,fr,bl,br,x_position,y_position,angle,x_velocity,y_velocity,angular_velocity
	 * </code>
	 */
	public static DriveRecord fromCSV(InputStream reader) throws IOException {
		var csv = CSVParser.parse(reader);

		TreeMap<Double, SystemState> states = new TreeMap<>();
		csv.forEach(record -> {
				double time = Double.parseDouble(record.get("time"));
			var pose = new Pose(
					Double.parseDouble(record.get("x_position")),
					Double.parseDouble(record.get("y_position")),
					Double.parseDouble(record.get("angle"))
				);

				var twist = new Twist(
					Double.parseDouble(record.get("x_velocity")),
					Double.parseDouble(record.get("y_velocity")),
					Double.parseDouble(record.get("angular_velocity"))
				).rotate(pose.orientation());

				var powers = MotorPowers.ofPowers(
					Double.parseDouble(record.get("fl")),
					Double.parseDouble(record.get("fr")),
					Double.parseDouble(record.get("bl")),
					Double.parseDouble(record.get("br"))
				);

				states.put(
					time,
					SystemState.ofLocalisationAndPowers(LocalisationDatum.of(pose, twist), powers)
				);
			}
		);

		return new DriveRecord(states);
	}

	public static DriveRecord zero() {
		TreeMap<Double, SystemState> states = new TreeMap<>();
		states.put(0., SystemState.ofLocalisationAndPowers(LocalisationDatum.zero(), MotorPowers.zero()));
		return new DriveRecord(states);
	}

	public DriveRecord subRecord(double minTime, double maxTime) {
		var newMap = new TreeMap<>(states.subMap(minTime, true, maxTime, true));
		if (minTime <= minTime())
			newMap.put(minTime, get(minTime));
		if (maxTime >= maxTime())
			newMap.put(maxTime, get(maxTime));
		return new DriveRecord(newMap);
	}

	public SystemState get(double time) {
		if (states.containsKey(time))
			return states.get(time);

		// search for lower and higher indices and do a linear interpolation/weighted average
		Double lower = states.floorKey(time);
		Double higher = states.ceilingKey(time);

		// it's impossible for both floor and ceiling to be null if the map is non-empty
		if (lower == null)
			return states.get(higher);

		if (higher == null)
			return states.get(lower);

		double timeDiff = higher - lower;
		double lowerWeight = (higher - time) / timeDiff;
		double higherWeight = (time - lower) / timeDiff;

		double[] lowerState = states.get(lower).toArray();
		double[] higherState = states.get(higher).toArray();

		double[] weightedAverage = new double[SystemState.SIZE];

		for (int i = 0; i < SystemState.SIZE; i++)
			weightedAverage[i] = lowerState[i] * lowerWeight + higherState[i] * higherWeight;

		return SystemState.fromDoubleArray(weightedAverage);
	}

	public double minTime() {
		return states.firstKey();
	}

	public double maxTime() {
		return states.lastKey();
	}

	public double getNearestTimeInRange(SystemState prevState, double minTime, double maxTime) {
		double bestTime = minTime;
		double bestDistance = Double.POSITIVE_INFINITY;

		for (var entry : states.subMap(minTime, true, maxTime, true).entrySet()) {
			double distance = entry.getValue().linearDistanceTo(prevState);
			if (distance < bestDistance) {
				bestTime = entry.getKey();
				bestDistance = distance;
			}
		}

		return bestTime;
	}

	public double nearestInterpolatedTimeInRange(SystemState prevState, double minTime, double maxTime, double step) {
		if (step <= 0)
			throw new IllegalArgumentException("step must be positive");

		if (minTime > maxTime())
			return maxTime();

		if (maxTime < minTime())
			return minTime();

		double bestTime = minTime;
		double bestDistance = Double.POSITIVE_INFINITY;

		for (double time = minTime; time <= maxTime; time += step) {
			double distance = get(time).linearDistanceTo(prevState);
			if (distance < bestDistance) {
				bestTime = time;
				bestDistance = distance;
			}
		}

		// evaluate maxTime as well
		double distance = get(maxTime).linearDistanceTo(prevState);
		if (distance < bestDistance)
			bestTime = maxTime;

		return Math.min(bestTime, maxTime);
	}

	public SystemState endPoint() {
		return states.lastEntry().getValue();
	}

	/**
	 * Mirrors the entire DriveRecord by creating a new instance
	 * and mirroring each entry using {@link SystemState#mirror()}.
	 */
	public DriveRecord mirror() {
		TreeMap<Double, SystemState> newStates = new TreeMap<>();
		this.states.forEach((time, state) -> newStates.put(time, state.mirror()));
		return new DriveRecord(newStates);
	}

	public static DriveRecord ofStationary(SystemState state) {
		TreeMap<Double, SystemState> states = new TreeMap<>();
		states.put(0., state);

		return new DriveRecord(states);
	}

	public static DriveRecord ofStationary(Pose pose) {
		return ofStationary(SystemState.ofLocalisationAndPowers(
			LocalisationDatum.of(pose, Twist.zero()), MotorPowers.zero()));
	}

	@Override
	public String toString() {
		return states.toString();
	}

	public static class Builder {
		private static final double MOVEMENT_SPEED = 1.5; // m/s
		private static final double ROTATION_SPEED = 4; // rad/s
		private static final double FAST_MOVEMENT_SPEED = 2.0; // m/s
		private static final double FAST_ROTATION_SPEED = 6; // rad/s

		private static final double SLOW_MOVEMENT_SPEED = 1; // m/s
		private static final double SLOW_ROTATION_SPEED = 2; // rad/s
		private final TreeMap<Double, SystemState> states = new TreeMap<>();
		private SystemState lastState;
		private double lastTime = 0;

		public Builder(SystemState startingState) {
			this.lastState = startingState;
			states.put( lastTime, startingState);
		}

		public Builder(Pose startingPose) {
			this(SystemState.ofLocalisationAndPowers(LocalisationDatum.of(startingPose, Twist.zero()), MotorPowers.zero()));
		}

		public Builder goTo(SystemState state) {
			double distance = lastState.linearDistanceTo(state);
			double rotation = Math.abs(lastState.getTheta() - state.getTheta());
			double timeTaken = Math.max(distance / MOVEMENT_SPEED, rotation / ROTATION_SPEED);

			lastTime += timeTaken;
			lastState = state;
			states.put(lastTime, state);
			return this;
		}

		public Builder goTo(Pose pose) {
			return goTo(SystemState.ofLocalisationAndPowers(LocalisationDatum.of(pose, Twist.zero()), MotorPowers.zero()));
		}

		public Builder goToQuickly(SystemState state) {
			double distance = lastState.linearDistanceTo(state);
			double rotation = Math.abs(lastState.getTheta() - state.getTheta());
			double timeTaken = Math.max(distance / FAST_MOVEMENT_SPEED, rotation / FAST_ROTATION_SPEED);

			lastTime += timeTaken;
			lastState = state;
			states.put(lastTime, state);
			return this;
		}

		public Builder goToQuickly(Pose pose) {
			return goToQuickly(SystemState.ofLocalisationAndPowers(LocalisationDatum.of(pose, Twist.zero()), MotorPowers.zero()));
		}

		public Builder goToSlowly(SystemState state) {
			double distance = lastState.linearDistanceTo(state);
			double rotation = Math.abs(lastState.getTheta() - state.getTheta());
			double timeTaken = Math.max(distance / SLOW_MOVEMENT_SPEED, rotation / SLOW_ROTATION_SPEED);

			lastTime += timeTaken;
			lastState = state;
			states.put(lastTime, state);
			return this;
		}

		public Builder goToSlowly(Pose pose) {
			return goToSlowly(SystemState.ofLocalisationAndPowers(LocalisationDatum.of(pose, Twist.zero()), MotorPowers.zero()));
		}

		public Builder addDelay(Duration delay) {
			lastTime += delay.toSeconds();
			states.put(lastTime, lastState);
			return this;
		}

		public DriveRecord build() {
			return new DriveRecord(states);
		}
	}
}
