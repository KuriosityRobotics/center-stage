package com.kuriosityrobotics.centerstage.localisation;

import static com.kuriosityrobotics.centerstage.util.Units.IN;

import static java.lang.Math.toRadians;

import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.bulkdata.RevHubBulkData;
import com.kuriosityrobotics.centerstage.cameras.AprilTagOdometryProcessor;
import com.kuriosityrobotics.centerstage.localisation.messages.LocalisationDatum;
import com.kuriosityrobotics.centerstage.math.Point;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;
import com.kuriosityrobotics.centerstage.mechanisms.DrivetrainNode;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.kuriosityrobotics.centerstage.util.Instant;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The <code>OdometryIntegrator</code> class is a node that takes in a stream of {@link Twist} and Angle messages and
 * publishes a stream of {@link Pose} messages containing the robot's position.
 */
public class OdometryIntegrator {

	private final Odometry odometry = new Odometry();
	private final CorrectedIMU imu;
	private final AtomicReference<LocalisationDatum> state;

	private volatile Instant lastPositionIntegration;

	public OdometryIntegrator(ScheduledExecutorService ses, CorrectedIMU imu, BulkDataFetcher bulkDataFetcher, AprilTagOdometryProcessor... processors) {
		this.imu = imu;
		state = new AtomicReference<>(LocalisationDatum.of(Pose.zero(), Twist.zero()));
		lastPositionIntegration = Instant.now();

		ses.scheduleAtFixedRate(() -> {
			var imuAngle = imu.getYaw();
			var imuAngularVelocity = imu.getYawVelocity();

			state.getAndUpdate(state -> {
				state = new LocalisationDatum(new Pose(state.pose().x(), state.pose().y(), imuAngle), new Twist(state.twist().x(), state.twist().y(), imuAngularVelocity));
				return state;
			});
		}, 0, 1000 / 50, TimeUnit.MILLISECONDS);

		for (var processor : processors) {
			ses.scheduleAtFixedRate(
				() -> processor.getPoseEstimate().ifPresent(this::softResetPosition),
				0, 1000 / 10, TimeUnit.MILLISECONDS);
		}

		bulkDataFetcher.addExpansionHubListener(this::updateOdometry);
	}

	public OdometryIntegrator(ScheduledExecutorService ses, CorrectedIMU imu, BulkDataFetcher bulkDataFetcher) {
		this.imu = imu;
		state = new AtomicReference<>(LocalisationDatum.of(Pose.zero(), Twist.zero()));
		lastPositionIntegration = Instant.now();

		ses.scheduleAtFixedRate(() -> {
			var imuAngle = imu.getYaw();
			var imuAngularVelocity = imu.getYawVelocity();

			state.getAndUpdate(state -> {
				state = new LocalisationDatum(new Pose(state.pose().x(), state.pose().y(), imuAngle), new Twist(state.twist().x(), state.twist().y(), imuAngularVelocity));
				return state;
			});
		}, 0, 1000 / 50, TimeUnit.MILLISECONDS);

		bulkDataFetcher.addExpansionHubListener(this::updateOdometry);
	}

	private void updateOdometry(RevHubBulkData data) {
		state.getAndUpdate(state -> {
			var now = Instant.now();

			var relVel = odometry.calculateOdometryRel(data, state.twist().angular());

			double dt = now.since(lastPositionIntegration).toSeconds();

			double xChange = relVel.x() * dt;
			double yChange = relVel.y() * dt;
			double angleChange = relVel.angular() * dt;

			// learn calculus
			// integral of rotation matrix
			Twist step = Twist.of(
				 xChange * expSin(angleChange) + yChange * expCos(angleChange),
				-xChange * expCos(angleChange) + yChange * expSin(angleChange),
				angleChange
			);

			var pose = state.pose().add(step.rotate(state.pose().orientation()));
			var velocity = relVel; // explicitly make it clear: this is the relative velocity

			lastPositionIntegration = now;

			return LocalisationDatum.of(pose, velocity);
		});
	}

	public void resetPosition(Pose resetPose) {
		imu.resetAngle(resetPose.orientation());
		state.getAndUpdate(state -> {
			lastPositionIntegration = Instant.now();
			return LocalisationDatum.of(resetPose, Twist.zero());
		});
	}

	private void softResetPosition(Pose resetPose) {
		imu.resetWrappedAngle(resetPose.orientation());
		state.getAndUpdate(state -> {
			if (state.twist().velocity() > 0.2
				|| Math.abs(state.twist().angular()) > 0.2) return state; // do nothing

			lastPositionIntegration = Instant.now();
			return LocalisationDatum.of(resetPose, state.twist());
		});
	}

	public LocalisationDatum getLocalisation() {
		return state.get();
	}

	public LocalisationDatum getGlobalLocalisation() {
		var relativeLocalisation = getLocalisation();
		return LocalisationDatum.of(relativeLocalisation.pose(), relativeLocalisation.twist().rotate(relativeLocalisation.pose().orientation()));
	}

	/**
	 * Returns the continuous value of the exponential sine function,
	 * defined as expSin(x) = sin(x) / x
	 */
	private double expSin(double x) {
		if (x == 0) return 1;
		return Math.sin(x) / x;
	}

	/**
	 * Returns the continuous value of the exponential cosine function,
	 * defined as expCos(x) = (cos(x) - 1) / x
	 */
	private double expCos(double x) {
		if (x == 0) return 0;
		return (Math.cos(x) - 1) / x;
	}

	private static final double DISTANCE_TOLERANCE = 0.02;
	private static final double VELOCITY_TOLERANCE = 0.05;
	private static final double ANGLE_TOLERANCE = toRadians(2);

	public void testRoutine(Tester tester, DrivetrainNode dt) throws InterruptedException {
		tester.header("[Odometry Integrator]");
		tester.info("Position", () -> getLocalisation().pose().toString());
		tester.info("Velocity", () -> getLocalisation().twist().toString());

		tester.instruct("Place robot on its side");

		tester.info("Resetting position [1, -2, 0]");
		resetPosition(new Pose(1, -2, 0));

		tester.automaticAssert("Position near?",
			getLocalisation().pose().distance(new Point(1, -2)) <= DISTANCE_TOLERANCE); // don't care about angle

		tester.automaticAssert("Velocity zero?",
			getLocalisation().twist().velocity() <= VELOCITY_TOLERANCE); // don't care about angle

		tester.info("Resetting position [0, 0, 0]");
		resetPosition(Pose.zero());

		tester.automaticAssert("Position zero?",
			getLocalisation().pose().distance(Point.zero()) <= DISTANCE_TOLERANCE); // don't care about angle

		tester.automaticAssert("Velocity zero?",
			getLocalisation().twist().velocity() <= VELOCITY_TOLERANCE); // don't care about angle

		tester.instruct("Roll the forwards-rolling wheel forward");
		tester.automaticAssert("Moved forward?", getLocalisation().pose().x() >= DISTANCE_TOLERANCE);
		tester.automaticAssert("Didn't move sideways?", Math.abs(getLocalisation().pose().y()) <= DISTANCE_TOLERANCE);

		resetPosition(Pose.zero());

		tester.instruct("Roll the sideways-rolling wheel leftward");
		tester.automaticAssert("Moved leftward?", getLocalisation().pose().y() >= DISTANCE_TOLERANCE);
		tester.automaticAssert("Didn't move forward?", Math.abs(getLocalisation().pose().x()) <= DISTANCE_TOLERANCE);

		tester.instruct("Place the robot flat at the corner of a tile");

		resetPosition(Pose.zero());

		tester.automaticAssert("Position zero?", getLocalisation().pose().distance(new Point(1, -2)) <= DISTANCE_TOLERANCE);
		tester.automaticAssert("Velocity zero?", getLocalisation().twist().velocity() <= VELOCITY_TOLERANCE);
		tester.automaticAssert("Angle zero?", Math.abs(getLocalisation().pose().orientation()) <= ANGLE_TOLERANCE);

		dt.setBrakeMode(DcMotor.ZeroPowerBehavior.FLOAT);

		tester.instruct("Move the robot forward by exactly 1 tile");
		tester.automaticAssert("Velocity zero?", getLocalisation().twist().velocity() <= VELOCITY_TOLERANCE);
		tester.automaticAssert("Angle zero?", Math.abs(getLocalisation().pose().orientation()) <= ANGLE_TOLERANCE);
		tester.automaticAssert("X position match?", Math.abs(getLocalisation().pose().x() - 23.75 * IN) <= DISTANCE_TOLERANCE);
		tester.automaticAssert("Y position zero?", Math.abs(getLocalisation().pose().y()) <= DISTANCE_TOLERANCE);
		dt.setBrakeMode(DcMotor.ZeroPowerBehavior.BRAKE);
	}
}
