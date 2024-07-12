package com.kuriosityrobotics.centerstage.mechanisms.intake;

import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.hardware.LinearMotorControl;
import com.kuriosityrobotics.centerstage.mechanisms.HardwareUtils;
import com.kuriosityrobotics.centerstage.util.Duration;
import com.kuriosityrobotics.centerstage.util.Instant;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeoutException;

import static com.kuriosityrobotics.centerstage.util.Units.CM;
import static java.lang.Math.min;

public class IntakeExtensionMotor extends LinearMotorControl {
	public static final double TICKS_PER_METER = 1320.39; // 145.11 ticks / spool circumference, 1320.39
	private final BulkDataFetcher.BulkDataNotifier notifier;
	private final DcMotorEx delegate;
	private double offset;
	private boolean isTemporarilyDisengaged;
	private HardwareUtils.MotorState prevState;

	public IntakeExtensionMotor(BulkDataFetcher.BulkDataNotifier notifier, DcMotorEx delegate) throws InterruptedException {
		this.notifier = notifier;
		this.delegate = delegate;

		delegate.setTargetPosition(delegate.getCurrentPosition());
		delegate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		delegate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		delegate.setPositionPIDFCoefficients(20);
		delegate.setVelocityPIDFCoefficients(1.5, 0.1, 0, 25);
		delegate.setPower(1);
		delegate.setCurrentAlert(25, CurrentUnit.AMPS);

		delegate.setMotorEnable();

		calibrate();
	}

	public void calibrate() throws InterruptedException {
		lock.lockInterruptibly();
		try {
			var savedState = HardwareUtils.saveMotorState(delegate);

			// drive it backwards till it hits the hardstop
			try {
				delegate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				delegate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				delegate.setPower(-0.4);

				Instant startTime = Instant.now();
				do {
					Thread.sleep(100);

					if (Instant.now().since(startTime).isGreaterThan(Duration.ofMillis(5000)))
						throw new RuntimeException("Intake calibration timed out");
				} while (Math.abs(getVelocityMeters()) > 0.003);

				// reset encoder
				delegate.setPower(0);
				Thread.sleep(200); // wait for mechanism to relax
				delegate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				updateOffsetToMatch(0);
				setTargetPositionMetres0(0);
			} finally {
				HardwareUtils.restoreMotorState(delegate, savedState);
			}
		} finally {
			lock.unlock();
		}
	}

	@Override
	protected boolean isBusy() {
		double positionError = getTargetPositionMeters() - getPositionMeters();
		if (isTemporarilyDisengaged)
			return Math.abs(getVelocityMeters()) > 0.02;
//				|| positionError > -0.02; // if its locked in or not

		return Math.abs(getVelocityMeters()) > 0.02
			|| Math.abs(positionError) > 0.05;
	}

	public void ensureMotorsEngaged() {
		if (isTemporarilyDisengaged) {
			isTemporarilyDisengaged = false;
			HardwareUtils.restoreMotorState(delegate, prevState);

			prevState = null;
		}
	}

	private void temporarilyDisengage() {
		if (!isTemporarilyDisengaged) {
			isTemporarilyDisengaged = true;
			prevState = HardwareUtils.saveMotorState(delegate);
			delegate.setPower(0);
		}
	}

	@Override
	protected void idle() throws InterruptedException {
		notifier.await();
	}

	@Override
	protected void setTargetPositionMetres0(double position) {
		delegate.setTargetPosition((int) (position * TICKS_PER_METER - offset));
	}

	@Override
	public double getPositionMeters() {
		return (delegate.getCurrentPosition() + offset) / TICKS_PER_METER;
	}

	private double getTargetPositionMeters() {
		return (delegate.getTargetPosition() + offset) / TICKS_PER_METER;
	}

	@Override
	public void updateOffsetToMatch(double currentPosition) throws InterruptedException {
		offset = currentPosition * TICKS_PER_METER - delegate.getCurrentPosition();
	}

	@Override
	public double getVelocityMeters() {
		return delegate.getVelocity() / TICKS_PER_METER;
	}

	// by the time this method returns, it should either (a) be in the correct position, or (b) throw an exception
	public void goToPosition(IntakeSlidePosition position) throws InterruptedException, TimeoutException {
		try {
			ensureMotorsEngaged();
			goToPosition(position.position);
		} catch (TimeoutException e) {
			if (getPositionMeters() > getTargetPositionMeters())
				recoverFromStuckClosing(position);
			else
				recoverFromStuckOpening(position);
		}
	}

	public void retractAndLock() throws InterruptedException, TimeoutException {
		goToPosition(IntakeSlidePosition.CLOSED); // close with PID first

//		temporarilyDisengage();
//		delegate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//		delegate.setPower(-0.4);

		try {
			temporarilyDisengage();
			delegate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			delegate.setPower(-0.4);
			super.goToPosition(IntakeSlidePosition.CLOSED.position); // yucky locking
		} catch (TimeoutException e) {
			goToPosition(IntakeSlidePosition.CLOSED); // fallback plan
		}
	}

	private void recoverFromStuckClosing(IntakeSlidePosition position) throws InterruptedException, TimeoutException {
		goToPosition(min(getPositionMeters() + 5 * CM, 80)); // maybe something got stuck in the mechanism??
		goToPosition(position.position);
	}

	private void recoverFromStuckOpening(IntakeSlidePosition position) throws InterruptedException, TimeoutException {
		goToPosition(IntakeSlidePosition.CLOSED.position);
		throw new TimeoutException("Couldn't open intake");
	}

	public enum IntakeSlidePosition {
		CLOSED(0 * CM),
		MEDIUM(50 * CM),
		FULL(65 * CM);

		private final double position;

		IntakeSlidePosition(double pos) {
			this.position = pos;
		}
	}
}
