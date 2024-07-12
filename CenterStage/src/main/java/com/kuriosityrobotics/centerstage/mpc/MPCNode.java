package com.kuriosityrobotics.centerstage.mpc;

import com.kuriosityrobotics.centerstage.drive.MotorPowers;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.localisation.messages.LocalisationDatum;
import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;
import com.kuriosityrobotics.centerstage.mechanisms.DrivetrainNode;
import com.kuriosityrobotics.centerstage.util.Duration;
import com.kuriosityrobotics.centerstage.util.Instant;
import com.kuriosityrobotics.centerstage.concurrent.PreemptibleLock;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import static com.kuriosityrobotics.centerstage.mpc.SolverOutput.NUM_STAGES;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import androidx.annotation.GuardedBy;

public class MPCNode {
	private final Logger logger = LoggerFactory.getLogger(MPCNode.class);
	private final DrivetrainNode drivetrainNode;
	private final OdometryIntegrator localisation;
	private final StableVoltageSensor batteryVoltageSensor;
	private final ReentrantLock lock = new ReentrantLock();
	private final Condition stateUpdateCondition = lock.newCondition();
	@GuardedBy("lock")
	private Follower follower = null;
	@GuardedBy("lock")
	private final SolverInputBuilder builder;
	@GuardedBy("lock")
	private FollowState followState;
	@GuardedBy("lock")
	private Instant timeStable = null;

	private final ScheduledExecutorService ses;

	public MPCNode(ScheduledExecutorService ses, DrivetrainNode drivetrainNode, OdometryIntegrator localisation, StableVoltageSensor batteryVoltageSensor) throws InterruptedException {
		this.ses = ses;
		this.drivetrainNode = drivetrainNode;
		this.localisation = localisation;
		this.batteryVoltageSensor = batteryVoltageSensor;

		lock.lockInterruptibly();
		try {
			followState = FollowState.FINISHED;
			builder = new SolverInputBuilder()
				.withDefaultParameters()
				.withWeights(pathEndWeights());
		} finally {
			lock.unlock();
		}

		logger.info("Constructing MPCNode " + hashCode());

		ses.scheduleAtFixedRate(this::runSolver, 0, 1000 / 20, java.util.concurrent.TimeUnit.MILLISECONDS);
	}

	private void runSolver() {
		lock.lock();

		try {
			if (follower == null) return;

			var l = localisation.getGlobalLocalisation();
			follower.correctPosition(l);

			for (int stage = 0; stage < NUM_STAGES; stage++) {
				var state = follower.getTargetState(stage * 0.05);
				builder.setTargetsFor(stage, DriveTargets.fromSystemState(state));
			}

			builder.withInitialGuesses(follower.getGuesses());

			if (follower.isFinishing(l)) {
				builder.withWeights(pathEndWeights());
			} else {
				builder.withWeights(defaultWeights());
			}

			var solver = builder
				.startingAt(l)
				.build(batteryVoltageSensor.getVoltage());

			var start = Instant.now();
			var result = solver.solve();
			var end = Instant.now();

			logger.info("Solved in " + (end.since(start).toMillis()) + "ms");

//			logger.info(result);

			if (result.getExitCode() != 1) {
				logger.warn("MPCNode solver error: ExitCode " + result.getExitCode());
//				dispatchPowers(SystemState.ofLocalisationAndPowers(l, MotorPowers.zero()));
//				return;
			}

			dispatchPowers(result.getStates()[0]); // only use first result

			if (follower == null)
				return;
			follower.incrementTime(0.05);
			follower.correctPosition(localisation.getGlobalLocalisation());

//			double cost = result.getSolutionInfo().getPobj();

			Instant now = Instant.now();

			if (follower.isStable(l)) {
				if (timeStable == null) timeStable = now;
			} else timeStable = null;

			if (follower.isFinished(l))
				setFollowState(FollowState.FINISHED);
			else if (timeStable != null && timeStable.since(now).isGreaterThan(Duration.ofMillis(1000)))
				setFollowState(FollowState.FINISHED);

			follower.setFromSolution(result);
		} finally {
			lock.unlock();
		}
	}

	private void dispatchPowers(SystemState nextState) {
		var expected = new LocalisationDatum(
			new Pose(nextState.getX(), nextState.getY(), nextState.getTheta()),
			new Twist(nextState.getXVel(), nextState.getYVel(), nextState.getThetaVel()).rotate(-nextState.getTheta())
		);

		logger.info(String.format("actual: %s%nexpected: %s", localisation.getGlobalLocalisation(), expected));

		var motorPowers = nextState.getMotorPowers();

		drivetrainNode.setMotorVoltages(motorPowers);
	}

	@GuardedBy("lock")
	private void setFollowState(FollowState followState) {
		this.followState = followState;
		stateUpdateCondition.signalAll();
	}

	private final PreemptibleLock followerLock = new PreemptibleLock();

	public void followPath(DriveRecord driveRecord) throws InterruptedException {
		followerLock.lock();
		try {
			followPath0(driveRecord);
			followPath0(DriveRecord.ofStationary(driveRecord.endPoint())); // position lock
		} finally {
			followerLock.unlock();
		}
	}

	@GuardedBy("followerLock")
	private void followPath0(DriveRecord driveRecord) throws InterruptedException {
		lock.lock();
		try {
			timeStable = null;
			follower = new Follower(driveRecord);
			setFollowState(FollowState.EN_ROUTE);

			while (followState != FollowState.FINISHED)
				stateUpdateCondition.await();
		} finally {
			lock.unlock();
		}
	}

	public void stopFollowing() {
		followerLock.lock();
		try {
			stopFollowing0();
		} finally {
			followerLock.unlock();
		}
	}

	@GuardedBy("followerLock")
	private void stopFollowing0() {
		lock.lock();
		try {
			follower = null;
			setFollowState(FollowState.FINISHED);
		} finally {
			lock.unlock();
		}
	}

	private static final double DEFAULT_WEIGHT = 75;

	private static DriveWeights defaultWeights() {
		return new DriveWeights(
			1000, 1000, 220,
			0, 0, 0,
			DEFAULT_WEIGHT
		);
	}

	private static DriveWeights pathEndWeights() {
		return new DriveWeights(
			1000, 1000, 200,
			0, 0, 0,
			 DEFAULT_WEIGHT
		);
	}

	private class Follower {
		private final DriveRecord driveRecord;
		@GuardedBy("lock")
		private double timeAlongPath;
		private SystemState[] guesses = new SystemState[NUM_STAGES];

		Follower(DriveRecord driveRecord) {
			this.driveRecord = driveRecord;
			this.timeAlongPath = driveRecord.minTime();
			for (int i = 0; i < NUM_STAGES; i++)
				guesses[i] = driveRecord.get(i * 0.05);
		}

		public SystemState targetState() {
			return driveRecord.get(timeAlongPath);
		}

		public void correctPosition(LocalisationDatum localisation) {
			SystemState currentState = SystemState.ofLocalisationAndPowers(localisation, MotorPowers.zero());
			double positionError = targetState().linearDistanceTo(currentState);
			if (positionError > 0.02)
				timeAlongPath = driveRecord.nearestInterpolatedTimeInRange(currentState, timeAlongPath - 0.04, timeAlongPath + 0.05, 0.005);
		}

		public void incrementTime(double timeIncrement) {
			timeAlongPath = Math.min(timeAlongPath + timeIncrement, driveRecord.maxTime());
		}

		public SystemState getTargetState(double timeAhead) {
			return driveRecord.get(timeAlongPath + timeAhead);
		}

		public boolean isFinishing(LocalisationDatum localisation) {
			SystemState currentState = SystemState.ofLocalisationAndPowers(localisation, MotorPowers.zero());
			double positionError = driveRecord.get(driveRecord.maxTime()).linearDistanceTo(currentState);
			return positionError < 1.0;
		}

		public boolean isFinished(LocalisationDatum localisation) {
			SystemState currentState = SystemState.ofLocalisationAndPowers(localisation, MotorPowers.zero());
			double positionError = targetState().linearDistanceTo(currentState);
			double angleError = Math.abs(localisation.pose().orientation() - targetState().getTheta());
			logger.info("Position Error: " + positionError);
			System.out.println("Time left: " + (driveRecord.maxTime() - timeAlongPath));
			logger.info("Angle Error: " + toDegrees(angleError));
			return driveRecord.maxTime() - timeAlongPath <= 0.5
				&& positionError < 0.03 && angleError < toRadians(3);
		}

		public boolean isStable(LocalisationDatum localisation) {
			SystemState currentState = SystemState.ofLocalisationAndPowers(localisation, MotorPowers.zero());
			double positionError = driveRecord.get(driveRecord.maxTime()).linearDistanceTo(currentState);
			double angleError = Math.abs(localisation.pose().orientation() - targetState().getTheta());
			double velocity = localisation.twist().velocity();
			double angularVelocity = localisation.twist().angular();

			return driveRecord.maxTime() - timeAlongPath <= 1
				&& positionError < 0.1 && angleError < toRadians(10)
				&& velocity < 0.1 && Math.abs(angularVelocity) < 0.1;
		}

		public SystemState[] getGuesses() {
			return guesses;
		}

		public void setFromSolution(SolverOutput solution) {
			guesses = solution.getStates();
		}
	}

	private enum FollowState {
		EN_ROUTE,
		FINISHED
	}
}