package com.kuriosityrobotics.centerstage.mechanisms.outtake;


import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.hardware.LinearMotorControl;
import com.kuriosityrobotics.centerstage.mechanisms.HardwareUtils;
import com.kuriosityrobotics.centerstage.util.Duration;
import com.kuriosityrobotics.centerstage.util.Instant;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

/**
 * A motor that controls the outtake.
 */
public class OuttakeMotor extends LinearMotorControl {
	private static final double TICKS_PER_METRE = 2064.04; // calculation: (145.1 / circumference) * (25 / 16)

	private final BulkDataFetcher.BulkDataNotifier notifier;
	final DcMotorEx delegate;

	private double offset;

	public OuttakeMotor(BulkDataFetcher.BulkDataNotifier notifier, DcMotorEx delegate) {
		super(Duration.ofSeconds(5));
		this.notifier = notifier;
		this.delegate = delegate;

		delegate.setTargetPosition(delegate.getCurrentPosition());
		delegate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		delegate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		delegate.setPositionPIDFCoefficients(20);
		delegate.setVelocityPIDFCoefficients(5, 0, 0, 12);
		delegate.setPower(1);

		delegate.setCurrentAlert(25, CurrentUnit.AMPS);

		delegate.setMotorEnable();
	}

	@Override
	public void updateOffsetToMatch(double currentPosition) {
		offset = currentPosition * TICKS_PER_METRE - delegate.getCurrentPosition();
	}


	@Override
	public double getPositionMeters() {
		return (delegate.getCurrentPosition() + offset) / TICKS_PER_METRE;
	}

	public double getTargetPositionMeters() {
		return (delegate.getTargetPosition() + offset) / TICKS_PER_METRE;
	}

	@Override
	public double getVelocityMeters() {
		return delegate.getVelocity() / TICKS_PER_METRE;
	}

	@Override
	protected void setTargetPositionMetres0(double position) {
		delegate.setTargetPosition((int) (position * TICKS_PER_METRE - offset));
	}

	@Override
	protected void idle() throws InterruptedException {
		notifier.await();
	}

	@Override
	protected boolean isBusy() {
//		return delegate.isBusy();
		double positionError = getTargetPositionMeters() - getPositionMeters();
		return Math.abs(getVelocityMeters()) > 0.01
			|| Math.abs(positionError) > 0.05;
	}

	public void calibrate() throws InterruptedException {
		lock.lockInterruptibly();
		try {
			var savedState = HardwareUtils.saveMotorState(delegate);

			// drive it downwards till it hits the hardstop
			try {
				delegate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
				delegate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				delegate.setPower(-0.2);

				Instant startTime = Instant.now();
				do {
					Thread.sleep(100);

					if (Instant.now().since(startTime).isGreaterThan(Duration.ofMillis(5000)))
						throw new RuntimeException("Outtake calibration timed out");
				} while (Math.abs(getVelocityMeters()) > 0.003);

				// reset encoder
				delegate.setPower(0);
				Thread.sleep(500); // wait for mechanism to relax
				delegate.setMode(STOP_AND_RESET_ENCODER);
				updateOffsetToMatch(0);
				setTargetPositionMetres0(0);
			} finally {
				HardwareUtils.restoreMotorState(delegate, savedState);
			}
		} finally {
			lock.unlock();
		}
	}
}