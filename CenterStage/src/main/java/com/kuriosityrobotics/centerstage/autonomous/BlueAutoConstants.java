package com.kuriosityrobotics.centerstage.autonomous;

import static com.kuriosityrobotics.centerstage.util.Units.CM;
import static java.lang.Math.toRadians;

import com.kuriosityrobotics.centerstage.cameras.SpikeLocation;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;
import com.kuriosityrobotics.centerstage.mpc.DriveRecord;

/**
 * Paths, points, and configs, all measured for the Blue side.
 * Use {@link Pose#mirror()} and {@link DriveRecord#mirror()} on these constants for blue side auto
 */
public class BlueAutoConstants {
	public static final Pose FIELD_START_POSE = new Pose(79 * CM, 23 * CM, toRadians(-90));
	public static final Pose BOARD_START_POSE = new Pose(216 * CM, 23 * CM, toRadians(-90));

	public static final double GUTTER_Y = 30 * CM;
	public static final double GATE_Y = 150 * CM;

	public static final double INTAKE_ALIGN_X = 96 * CM;
	public static final double INTAKE_X_3 = 103 * CM;
	public static final double INTAKE_X_3_OTHER = 87 * CM;
	public static final double INTAKE_1_Y = 39 * CM;
	public static final double INTAKE_2_Y = 129.5 * CM;
	public static final double INTAKE_3_Y = 145 * CM;

	public static final Pose CORNER = new Pose(290 * CM, GUTTER_Y, toRadians(-180));
	public static final Pose BACKSTAGE = new Pose(290 * CM, GATE_Y, toRadians(-180));
	public static final Pose BOARD_POSE_LEFT = new Pose(303 * CM, 79 * CM, toRadians(-180));
	public static final Pose BOARD_POSE_CENTER = new Pose(303 * CM, 99 * CM, toRadians(-180));
	public static final Pose BOARD_POSE_RIGHT = new Pose(303 * CM, 116 * CM, toRadians(-180));

	public static final Pose INTAKE_POSE_1 = new Pose(INTAKE_ALIGN_X, INTAKE_1_Y, toRadians(-180));
	public static final Pose INTAKE_POSE_2 = new Pose(INTAKE_ALIGN_X, INTAKE_2_Y, toRadians(-180));
	public static final Pose INTAKE_POSE_3 = new Pose(INTAKE_X_3, INTAKE_3_Y, toRadians(-182));
	public static final Pose INTAKE_POSE_3_OTHER = new Pose(INTAKE_X_3_OTHER, INTAKE_3_Y, toRadians(-180));

	public static final Pose GUTTER_JUNCTION = new Pose(89 * CM, GUTTER_Y, toRadians(-180));
	public static final Pose GATE_JUNCTION = new Pose(89 * CM, GATE_Y, toRadians(-180));

	public static final DriveRecord FIELD_START_TO_CORNER = new DriveRecord.Builder(FIELD_START_POSE)
		.goTo(new Pose(78 * CM, 28 * CM, toRadians(-90)))
		.goTo(new Pose(78 * CM, GUTTER_Y, toRadians(-180)))
		.goToQuickly(new Pose(230 * CM, GUTTER_Y, toRadians(-180)))
		.goTo(CORNER)
		.build();

	public static final DriveRecord BOARD_START_TO_CORNER = new DriveRecord.Builder(BOARD_START_POSE)
		.goTo(new Pose(222 * CM, 28 * CM, toRadians(-180)))
		.goTo(new Pose(222 * CM, GUTTER_Y, toRadians(-180)))
		.goToQuickly(new Pose(230 * CM, GUTTER_Y, toRadians(-180)))
		.goTo(CORNER)
		.build();


	public static final DriveRecord CORNER_TO_BOARD = new DriveRecord.Builder(CORNER)
		.goTo(new Pose(280 * CM, GUTTER_Y, toRadians(-180)))
		.goTo(new Pose(280 * CM, 70 * CM, toRadians(-180)))
		.goToSlowly(BOARD_POSE_CENTER)
		.build();

	public static final DriveRecord BOARD_TO_CORNER = new DriveRecord.Builder(BOARD_POSE_CENTER)
		.goTo(new Pose(280 * CM, 70 * CM, toRadians(-180)))
		.goTo(new Pose(280 * CM, GUTTER_Y, toRadians(-180)))
		.goTo(CORNER)
		.build();

	public static final DriveRecord CORNER_TO_CYCLE = new DriveRecord.Builder(CORNER)
		.goToQuickly(GUTTER_JUNCTION)
		.goToSlowly(INTAKE_POSE_1)
		.build();

	public static final DriveRecord CYCLE_TO_CORNER = new DriveRecord.Builder(INTAKE_POSE_1)
		.goTo(GUTTER_JUNCTION)
		.goToQuickly(CORNER)
		.build();

	//gate paths
	public static final DriveRecord FIELD_START_TO_BACKSTAGE = new DriveRecord.Builder(FIELD_START_POSE)
		.goTo(new Pose(78 * CM, GATE_Y, toRadians(-180)))
		.goToQuickly(new Pose(230 * CM, GATE_Y, toRadians(-180)))
		.goTo(BACKSTAGE)
		.build();

	public static final DriveRecord BOARD_START_TO_BACKSTAGE = new DriveRecord.Builder(BOARD_START_POSE)
		.goTo(new Pose(220.5 * CM, GATE_Y, toRadians(-180)))
		.goToQuickly(new Pose(230 * CM, GATE_Y, toRadians(-180)))
		.goTo(BACKSTAGE)
		.build();

	public static final DriveRecord BACKSTAGE_TO_BOARD = new DriveRecord.Builder(BACKSTAGE)
		.goTo(new Pose(280 * CM, GATE_Y, toRadians(-180)))
		.goTo(new Pose(280 * CM, 89 * CM, toRadians(-180)))
		.goToSlowly(BOARD_POSE_CENTER)
		.build();

	public static final DriveRecord BOARD_TO_BACKSTAGE = new DriveRecord.Builder(BOARD_POSE_CENTER)
		.goTo(new Pose(280 * CM, 89 * CM, toRadians(-180)))
		.goTo(new Pose(280 * CM, GATE_Y, toRadians(-180)))
		.goTo(BACKSTAGE)
		.build();

	public static final DriveRecord BACKSTAGE_TO_CYCLE = new DriveRecord.Builder(BACKSTAGE)
		.goToQuickly(GATE_JUNCTION)
		.goToSlowly(INTAKE_POSE_3)
		.build();

	public static final DriveRecord BACKSTAGE_TO_CYCLE_OTHER = new DriveRecord.Builder(BACKSTAGE)
		.goTo(GATE_JUNCTION)
		.goToSlowly(INTAKE_POSE_3_OTHER)
		.build();

	public static final DriveRecord CYCLE_TO_BACKSTAGE = new DriveRecord.Builder(INTAKE_POSE_3)
		.goTo(GATE_JUNCTION)
		.goToQuickly(BACKSTAGE)
		.build();

	// spike paths
	public static final double CENTER_Y = 87 * CM;
	public static final double HORIZONTAL_SPIKE_Y = 85 * CM;

	public static final Pose FIELD_RIGHT_SPIKE = new Pose(86 * CM, HORIZONTAL_SPIKE_Y, toRadians(-30));
	public static final Pose FIELD_CENTER_SPIKE = new Pose(89 * CM, CENTER_Y, toRadians(-90));
	public static final Pose FIELD_LEFT_SPIKE = new Pose(95 * CM, HORIZONTAL_SPIKE_Y, toRadians(-150));

	public static final Pose BOARD_RIGHT_SPIKE = new Pose(205 * CM, HORIZONTAL_SPIKE_Y, toRadians(-30));
	public static final Pose BOARD_CENTER_SPIKE = new Pose(211 * CM, CENTER_Y, toRadians(-90));
	public static final Pose BOARD_LEFT_SPIKE = new Pose(214 * CM, HORIZONTAL_SPIKE_Y, toRadians(-150));

	public static final Pose FIELD_SPIKE_JUNCTION = new Pose(89 * CM, 75 * CM, toRadians(-90));
	public static final Pose BOARD_SPIKE_JUNCTION = new Pose(210 * CM, 75 * CM, toRadians(-90));

	public static final Pose FIELD_CENTER_SPIKE_FOR_GATE = new Pose(89 * CM, 146 * CM, toRadians(180));

	private static Pose fieldCoordinateFor(SpikeLocation spike, boolean gate) {
		switch (spike){
			case LEFT:
				return FIELD_LEFT_SPIKE;
			case CENTER:
				return gate ? FIELD_CENTER_SPIKE_FOR_GATE : FIELD_CENTER_SPIKE;
			default:
				return FIELD_RIGHT_SPIKE;
		}
	}

	private static Pose boardCoordinateFor(SpikeLocation spike) {
		switch (spike){
			case LEFT:
				return BOARD_LEFT_SPIKE;
			case CENTER:
				return BOARD_CENTER_SPIKE;
			default:
				return BOARD_RIGHT_SPIKE;
		}
	}


	public static DriveRecord fieldSpikeToGate(SpikeLocation spike){
		DriveRecord.Builder dr = new DriveRecord.Builder(fieldCoordinateFor(spike, true));

		if(spike.equals(SpikeLocation.CENTER) || spike.equals(SpikeLocation.LEFT)){
			return dr.goTo(GATE_JUNCTION).build();
		} else {
			return dr.goToSlowly(GATE_JUNCTION).build();
		}
	}

	public static DriveRecord fieldToSpikeForGate(SpikeLocation spike){
		DriveRecord.Builder builder = new DriveRecord.Builder(FIELD_START_POSE);

		Pose spikePosition = fieldCoordinateFor(spike, true);
		Twist gameElementPush = fieldGameElementOffset(spike);

		builder
			.goTo(FIELD_SPIKE_JUNCTION)
			.goTo(spikePosition.add(gameElementPush))
			.goToSlowly(spikePosition);

		return builder.build();
	}

	public static DriveRecord fieldToSpike(SpikeLocation spike){
		DriveRecord.Builder builder = new DriveRecord.Builder(FIELD_START_POSE);

		Pose spikePosition = fieldCoordinateFor(spike, false);
		Twist gameElementPush = fieldGameElementOffset(spike);

		builder
			.goTo(FIELD_SPIKE_JUNCTION)
			.goTo(spikePosition.add(gameElementPush))
			.goToSlowly(spikePosition);

		return builder.build();
	}

	private static Twist fieldGameElementOffset(SpikeLocation spike) {
		switch (spike){
			case LEFT:
				return new Twist(20 * CM, 0, toRadians(-30));
			case CENTER:
				return new Twist(0, 20 * CM, 0);
			default:
				return new Twist(-20 * CM, 0, 0);
		}
	}

	private static Twist boardGameElementOffset(SpikeLocation spike) {
		switch (spike){
			case LEFT:
				return new Twist(20 * CM, 0, 0);
			case CENTER:
				return new Twist(0, 20 * CM, 0);
			default:
				return new Twist(-20 * CM, 0, toRadians(30));
		}
	}

	public static DriveRecord boardToSpike(SpikeLocation spike){
		DriveRecord.Builder builder = new DriveRecord.Builder(BOARD_START_POSE);

		Pose spikePosition = boardCoordinateFor(spike);
		Twist gameElementPush = boardGameElementOffset(spike);

		builder
			.goTo(BOARD_SPIKE_JUNCTION)
			.goTo(spikePosition.add(gameElementPush))
			.goToSlowly(spikePosition);

		return builder.build();
	}

	public static DriveRecord fieldSpikeToCorner(SpikeLocation spike) {
		Pose spikePosition = fieldCoordinateFor(spike, false);
		return new DriveRecord.Builder(spikePosition)
			.goTo(FIELD_SPIKE_JUNCTION.withAngle(spikePosition.orientation()))
			.goTo(FIELD_SPIKE_JUNCTION)
			.goToSlowly(new Pose(85 * CM, GUTTER_Y, toRadians(-180)))
			.goTo(new Pose(230 * CM, GUTTER_Y, toRadians(-180)))
			.goToQuickly(CORNER)
			.build();
	}

	public static Pose depositPositionFor(SpikeLocation spike){
		switch (spike) {
			case LEFT:
				return BOARD_POSE_LEFT;
			case CENTER:
				return BOARD_POSE_CENTER;
			default:
				return BOARD_POSE_RIGHT;
		}
	}

	public static DriveRecord cornerToDeposit(SpikeLocation spike) {
		Pose pose = depositPositionFor(spike);
		return new DriveRecord.Builder(CORNER)
			.goTo(new Pose(280 * CM, GUTTER_Y, toRadians(-180)))
			.goTo(new Pose(280 * CM, 70 * CM, toRadians(-180)))
			.goToSlowly(pose)
			.build();
	}

	public static DriveRecord depositToCorner(SpikeLocation spike) {
		Pose pose = depositPositionFor(spike);
		return new DriveRecord.Builder(pose)
			.goTo(new Pose(280 * CM, 70 * CM, toRadians(-180)))
			.goToSlowly(new Pose(280 * CM, GUTTER_Y, toRadians(-180)))
			.goToQuickly(CORNER)
			.build();
	}

	public static DriveRecord depositToBackstage(SpikeLocation spike) {
		Pose pose = depositPositionFor(spike);
		return new DriveRecord.Builder(pose)
			.goTo(new Pose(280 * CM, pose.y(), toRadians(-180)))
			.goToSlowly(new Pose(280 * CM, pose.y(), toRadians(-180)))
			.goToQuickly(BACKSTAGE)
			.build();
	}

	public static DriveRecord boardSpikeToCorner(SpikeLocation spike){
		Pose spikePosition = boardCoordinateFor(spike);
		return new DriveRecord.Builder(spikePosition)
			.goTo(BOARD_SPIKE_JUNCTION.withAngle(spikePosition.orientation()))
			.goTo(BOARD_SPIKE_JUNCTION)
			.goTo(new Pose(220 * CM, GUTTER_Y, toRadians(-180)))
			.goTo(new Pose(230 * CM, GUTTER_Y, toRadians(-180)))
			.goTo(CORNER)
			.build();
	}

	public static DriveRecord boardSpikeToDeposit(SpikeLocation spike){
		Pose spikePosition = boardCoordinateFor(spike);
		Pose depositPosition = depositPositionFor(spike);

		return new DriveRecord.Builder(spikePosition)
			.goTo(BOARD_SPIKE_JUNCTION.withAngle(spikePosition.orientation()))
			.goTo(BOARD_SPIKE_JUNCTION)
			.goTo(new Pose(220 * CM, GUTTER_Y, toRadians(-180)))
			.goTo(new Pose(230 * CM, GUTTER_Y, toRadians(-180)))
			.goTo(new Pose(280 * CM, 99 * CM, toRadians(-180)))
			.goToSlowly(depositPosition)
			.build();
	}

	public static DriveRecord fieldSpikeToBackstage(SpikeLocation spike){
		// todo: use if conditionals: if centre, must drive around, or smth
		Pose spikePosition = fieldCoordinateFor(spike, false);
		return new DriveRecord.Builder(spikePosition)
			.goTo(FIELD_SPIKE_JUNCTION.withAngle(spikePosition.orientation()))
			.goTo(FIELD_SPIKE_JUNCTION)
			.goTo(new Pose(78 * CM, GATE_Y, toRadians(-180)))
			.goTo(new Pose(230 * CM, GATE_Y, toRadians(-180)))
			.goTo(BACKSTAGE)
			.build();
	}

	public static DriveRecord boardSpikeToBackstage(SpikeLocation spike){
		Pose spikePosition = boardCoordinateFor(spike);
		return new DriveRecord.Builder(spikePosition)
			.goTo(BOARD_SPIKE_JUNCTION.withAngle(spikePosition.orientation()))
			.goTo(BOARD_SPIKE_JUNCTION)
			.goTo(new Pose(220.5 * CM, GATE_Y, toRadians(-180)))
			.goTo(new Pose(230 * CM, GATE_Y, toRadians(-180)))
			.goTo(BACKSTAGE)
			.build();
	}
}
