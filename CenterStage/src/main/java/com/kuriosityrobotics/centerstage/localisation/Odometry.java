package com.kuriosityrobotics.centerstage.localisation;

import com.kuriosityrobotics.centerstage.bulkdata.RevHubBulkData;
import com.kuriosityrobotics.centerstage.math.Twist;

import static com.kuriosityrobotics.centerstage.util.Units.CM;
import static java.lang.Math.PI;

/**
 * The <code>Odometry</code> class is a node that takes in a stream of {@link RevHubBulkData} and Angular Velocity messages and
 * publishes a stream of {@link Twist} messages containing the robot's velocity.
 */
public class
Odometry {
	/**
	 * The port number of the forwards-rolling encoder.
	 */
	public static final int FORWARDS_ODO_PORT = 0;
	/**
	 * The port number of the sideways-rolling encoder.
	 */
	public static final int SIDEWAYS_ODO_PORT = 3;

	/**
	 * The radius of the robot's odometry wheels.
	 */
	public static final double WHEEL_RADIUS = 1.75 * CM;
	/**
	 * The leftwards distance from the center of the robot to the forwards-rolling encoder.
	 */
	public static final double DISTANCE_TO_FORWARDS_ENCODER = -13.8025 * CM;
	/**
	 * The forwards distance from the center of the robot to the sideways-rolling encoder.
	 */
	public static final double DISTANCE_TO_SIDEWAYS_ENCODER = 4.7254 * CM;

	public Twist calculateOdometryRel(RevHubBulkData bulkData, double angularVel) {
		var forwardsRollingVel = 2 * PI * bulkData.velocities[FORWARDS_ODO_PORT] / 8192.;
		var sidewaysRollingVel = -2 * PI * bulkData.velocities[SIDEWAYS_ODO_PORT] / 8192.;

		// learn geometry
		double relativeVelForwards = WHEEL_RADIUS * forwardsRollingVel + DISTANCE_TO_FORWARDS_ENCODER * angularVel;
		double relativeVelSideways = WHEEL_RADIUS * sidewaysRollingVel - DISTANCE_TO_SIDEWAYS_ENCODER * angularVel;

		return new Twist(relativeVelForwards, relativeVelSideways, angularVel);
	}
}
