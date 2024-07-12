package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;
import com.kuriosityrobotics.centerstage.mechanisms.HardwareUtils;
import com.kuriosityrobotics.centerstage.test.Tester;
import com.kuriosityrobotics.centerstage.concurrent.PreemptibleLock;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.TimeoutException;

import static com.kuriosityrobotics.centerstage.util.Units.CM;
import static java.lang.Math.*;

public class OuttakeExtensionSlides {
	/**
	 * The incline angle of the robot's outtake linear slides from horizontal, in radians
	 */
	private static final double SLIDES_ANGLE = toRadians(60);
	private static final double BASE_HEIGHT = 28 * CM;

	private final Logger logger = LoggerFactory.getLogger(OuttakeExtensionSlides.class);
	private final PreemptibleLock slidesLock = new PreemptibleLock();

	private final OuttakeMotor upperMotor;
	private final OuttakeMotor lowerMotor;

	private boolean isTemporarilyDisengaged;
	private HardwareUtils.MotorState prevLowerState, prevUpperState;

	public OuttakeExtensionSlides(OuttakeMotor upperMotor, OuttakeMotor lowerMotor) throws InterruptedException {
		this.upperMotor = upperMotor;
		this.lowerMotor = lowerMotor;

		calibrate();
	}

	public void goToPositions(OuttakeSlidePosition position) throws InterruptedException, SlidesJammed {
		var oldPosition = lowerMotor.getPositionMeters();
		slidesLock.lock();
		try {
			ensureMotorsEngaged();
			goToPositions0(position.position);
		} catch (TimeoutException e) {
			handleJam(position, oldPosition);
		} finally {
			slidesLock.unlock();
		}
	}

	public void calibrate() throws InterruptedException {
		var oldRight = HardwareUtils.saveMotorState(lowerMotor.delegate);
		var oldLeft = HardwareUtils.saveMotorState(upperMotor.delegate);

		try (var scope = HardwareTaskScope.open()) {
			lowerMotor.delegate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			upperMotor.delegate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			lowerMotor.delegate.setPower(0);
			upperMotor.delegate.setPower(0);

			scope.fork(upperMotor::calibrate);
			scope.fork(lowerMotor::calibrate);
			scope.join();
		} finally {
			HardwareUtils.restoreMotorState(lowerMotor.delegate, oldRight);
			HardwareUtils.restoreMotorState(upperMotor.delegate, oldLeft);
		}
	}

	private void handleJam(OuttakeSlidePosition position, double oldPosition) throws InterruptedException, SlidesJammed {
		if (lowerMotor.getPositionMeters() < lowerMotor.getTargetPositionMeters()) {
			handleExtensionJam(oldPosition);
		} else {
			handleRetractionJam(position, oldPosition);
		}
	}

	private void handleRetractionJam(OuttakeSlidePosition position, double oldPosition) throws InterruptedException, SlidesJammed {
		logger.error("Can't retract outtake slides;  trying to move it out then in again to dislodge obstruction");
		try {
			goToPositions0(min(lowerMotor.getPositionMeters() + 5 * CM, 80)); // maybe something got stuck in the mechanism??
			goToPositions0(position.position); // try again, otherwise don't bother handling
			logger.warn("Recovered from jammed slides during retraction");
		} catch (InterruptedException | TimeoutException e2) {
			logger.error("gave up recovering;  going to previous position " + oldPosition);

			try {
				goToPositions0(oldPosition);
			} catch (TimeoutException e) {
				temporarilyDisengage();
				throw new SlidesJammed("Slides completely stuck. Temporarily disengaged motors till further command received", false);
			}

			throw new SlidesJammed("retraction failed: returned to previous position", false);
		}
	}

	private void handleExtensionJam(double oldPosition) throws InterruptedException, SlidesJammed {
		// bad, very bad!  this means that we've hit a ceiling or something
		logger.error("Outtake slides hit ceiling;  trying to go to previous position " + oldPosition);

		try {
			goToPositions0(oldPosition);
		} catch (TimeoutException e) {
			temporarilyDisengage();
			throw new SlidesJammed("Slides completely stuck. Temporarily disengaged motors till further command received", true);
		}
		throw new SlidesJammed("extension failed: returned to previous position", true);
	}

	private void goToPositions0(double position) throws InterruptedException, TimeoutException {
		try (var scope = HardwareTaskScope.open(TimeoutException.class)) {
			scope.fork(() -> upperMotor.goToPosition(position));
			scope.fork(() -> lowerMotor.goToPosition(position));

			scope.join();
		}
	}

	private void ensureMotorsEngaged() {
		if (isTemporarilyDisengaged) {
			isTemporarilyDisengaged = false;
			HardwareUtils.restoreMotorState(lowerMotor.delegate, prevLowerState);
			HardwareUtils.restoreMotorState(upperMotor.delegate, prevUpperState);

			prevLowerState = null;
			prevUpperState = null;
		}
	}

	private void temporarilyDisengage() {
		if (!isTemporarilyDisengaged) {
			isTemporarilyDisengaged = true;
			prevLowerState = HardwareUtils.saveMotorState(lowerMotor.delegate);
			prevUpperState = HardwareUtils.saveMotorState(upperMotor.delegate);
			lowerMotor.delegate.setPower(0);
			upperMotor.delegate.setPower(0);
		}
	}

	public void runRigging() {
		slidesLock.lock();
		try {
			temporarilyDisengage();
			upperMotor.delegate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			upperMotor.delegate.setPower(-1);
			lowerMotor.delegate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			lowerMotor.delegate.setPower(-1);
		} finally {
			slidesLock.unlock();
		}
	}

	public void testRoutine(Tester tester) throws InterruptedException, SlidesJammed {
		tester.header("[Outtake extension slides]");

		goToPositions(OuttakeSlidePosition.RETRACTED);

		tester.automaticAssert("motor should be stationary once goToPositions returns", abs(upperMotor.delegate.getVelocity()) < 10 && abs(lowerMotor.delegate.getVelocity()) < 10);
		tester.automaticAssert("total retracted motor current draw is less than 0.2A", upperMotor.delegate.getCurrent(CurrentUnit.AMPS) + lowerMotor.delegate.getCurrent(CurrentUnit.AMPS) < 0.2);
		tester.confirmThat("slides are retracted");

		goToPositions(OuttakeSlidePosition.MAX_HEIGHT);
		tester.automaticAssert("motor should be stationary once goToPositions returns", abs(upperMotor.delegate.getVelocity()) < 10 && abs(lowerMotor.delegate.getVelocity()) < 10);
		tester.confirmThat("slides are extended");
	}

	/**
	 * This enum represents the positions of the slides. The position is the distance the slides are extended, in metres.
	 */
	public enum OuttakeSlidePosition {
		RETRACTED(0),
		LOW_ROW(0.1),
		OVER_PASS(0.2),
		MEDIUM_HEIGHT(0.3),
		MEDIUM_HIGH(0.5),
		MAX_HEIGHT(0.6);

		/**
		 * The distance the slides are extended, in metres.
		 * This does not include the length of the slides when they are retracted.
		 */
		final double position;

		/**
		 * @param position the distance the slides are extended (diagonally), in metres
		 */
		OuttakeSlidePosition(double position) {
			this.position = position;
		}

		public boolean requiresOverPass() {
			return position < OVER_PASS.position;
		}

		public static double calculateExtensionLength(double height) {
			return (height - BASE_HEIGHT) / Math.sin(SLIDES_ANGLE);
		}
	}

	public static final class SlidesJammed extends Exception {
		public final Cause cause;

		public SlidesJammed(String message, boolean occurredWhileExtending) {
			super(message);
			this.cause = occurredWhileExtending ? Cause.EXTENSION : Cause.RETRACTION;
		}

		public enum Cause {
			EXTENSION,
			RETRACTION
		}
	}
}