package com.kuriosityrobotics.centerstage.mechanisms;

import static com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeSpeed.FAST;

import androidx.annotation.GuardedBy;

import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;
import com.kuriosityrobotics.centerstage.mechanisms.intake.Intake;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeExtensionMotor;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeLiftServo;
import com.kuriosityrobotics.centerstage.mechanisms.intake.IntakeSpeed;
import com.kuriosityrobotics.centerstage.mechanisms.outtake.Outtake;
import com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeExtensionSlides;

import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.ReentrantLock;

public class MechanismManager {
	private final ReentrantLock intakeLock = new ReentrantLock();
	private final ReentrantLock outtakeLock = new ReentrantLock();

	@GuardedBy("intakeLock")
	private final Intake intake;
	@GuardedBy("outtakeLock")
	private final Outtake outtake;

	public MechanismManager(Intake intake, Outtake outtake){
		this.intake = intake;
		this.outtake = outtake;
	}

	public void setIntakeSpeed(IntakeSpeed speed){
		if(outtake.armResting() && !intake.containerFlipped()){
			intake.setIntakeSpeed(speed);
		} else {
			intake.setIntakeSpeed(IntakeSpeed.STOP);
		}
	}

	@GuardedBy("intakeLock")
	private void flipIntake0() throws InterruptedException {
		intake.flip();
	}

	public void unflipIntake() throws InterruptedException {
		if (!intakeLock.tryLock()) throw new IllegalStateException("intake is busy!");
		try {
			intake.unflip();
		} finally {
			intakeLock.unlock();
		}
	}

	public void toPurplePreload() throws InterruptedException {
		if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			if (!intake.containerFlipped() && outtake.armResting())
				throw new IllegalStateException("cannot move to deposit height unless container flipped and outtake arm resting");

			if (outtake.armResting())
				intake.flipStep(); // if the arm is resting, we need to adjust the intake

			outtake.toPurplePreload();
		} finally {
			outtakeLock.unlock();
		}
	}

	public void depositPurple() throws InterruptedException {
		if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			outtake.depositPurple();
		} finally {
			outtakeLock.unlock();
		}
	}

	public void dropIntake() throws InterruptedException {
		setIntakeSpeed(FAST);
		intake.goToHeight(IntakeLiftServo.IntakeHeight.LIFTED);
		intake.goToHeight(IntakeLiftServo.IntakeHeight.STACK_FIVE);
		Thread.sleep(500);
//		setIntakeSpeed(IntakeSpeed.REVERSE);
//		Thread.sleep(100);
//		setIntakeSpeed(FAST);
		intake.goToHeight(IntakeLiftServo.IntakeHeight.STACK_FOUR);
		Thread.sleep(1000);
//		setIntakeSpeed(IntakeSpeed.REVERSE);
//		Thread.sleep(300);
//		setIntakeSpeed(FAST);
//		Thread.sleep(500);
//		setIntakeSpeed(STOP);
	}

	public void extendIntake(IntakeExtensionMotor.IntakeSlidePosition position) throws InterruptedException {
		System.out.println("got the intake lock to extend");
		if (!intakeLock.tryLock()) throw new IllegalStateException("intake is busy!");
		try (var scope = HardwareTaskScope.open(TimeoutException.class)) {
			scope.fork(intake::unflip);
 			scope.fork(() -> intake.goTo(position));

            scope.join();
		} catch (TimeoutException e) {
			throw new RuntimeException(e);
		} finally {
			intakeLock.unlock();
		}
	}

	public void stopIntake() {
		setIntakeSpeed(IntakeSpeed.STOP);
	}

	public void transfer() throws InterruptedException {
		if (!intakeLock.tryLock()) throw new IllegalStateException("intake is busy!");
		try {
			if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
			try {
				if (!outtake.armResting()) throw new IllegalStateException("cannot transfer while arm is not resting");

				attemptContainerFlip();
				outtake.latchClaws();
			} finally {
				outtakeLock.unlock();
			}
		} finally {
			intakeLock.unlock();
		}
	}

	private void attemptContainerFlip() throws InterruptedException {
		try (var scope = HardwareTaskScope.open(TimeoutException.class)) {
			scope.fork(outtake::transferArm);
			scope.fork(outtake::unlatchClaws);
			scope.fork(intake::retractAndLock);
			scope.join();

			flipIntake0();
		} catch (TimeoutException e) {
			throw new RuntimeException("Failed to retract intake", e);
		}
	}

	public void toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition position) throws InterruptedException {
		if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			if (!intake.containerFlipped() && outtake.armResting())
				throw new IllegalStateException("cannot move to deposit height unless container flipped and outtake arm resting");

			if (outtake.armResting())
				intake.flipStep(); // if the arm is resting, we need to adjust the intake

			outtake.toDepositHeight(position);
		} finally {
			outtakeLock.unlock();
		}
	}

	public void incrementHeight() throws InterruptedException {
		if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			if (!intake.containerFlipped() && outtake.armResting())
				throw new IllegalStateException("cannot move to deposit height unless container flipped");

			if (outtake.armResting()){
				intake.flipStep(); // if the arm is resting, we need to adjust the intake

				// due to multiple positions possibly being lower than the over_pass height,
				// assume that we want the lowest that is over it
				outtake.toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.OVER_PASS);
			} else {
				outtake.incrementHeight();
			}
		} finally {
			outtakeLock.unlock();
		}
	}

	public void decrementHeight() throws InterruptedException {
		if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			if (!intake.containerFlipped() && outtake.armResting())
				throw new IllegalStateException("cannot move to deposit height unless container flipped");

			// paranoid, this case never actually happens
			if (outtake.armResting())
				intake.flipStep(); // if the arm is resting, we need to adjust the intake

			outtake.decrementHeight();
		} finally {
			outtakeLock.unlock();
		}
	}

	public void releaseAndCollapse() throws InterruptedException {
		if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try (var scope = HardwareTaskScope.open()) {
			scope.fork(outtake::releaseAndCollapse);
			scope.fork(intake::unflip);

			scope.join();
		} finally {
			outtakeLock.unlock();
		}
	}

	public void toIntakeHeight(IntakeLiftServo.IntakeHeight height) throws InterruptedException {
		intake.goToHeight(height); // doesn't require lock
	}

	public void incrementWrist() throws InterruptedException {
		outtake.incrementWrist(); // doesn't require lock
	}

	public void decrementWrist() throws InterruptedException {
		outtake.decrementWrist(); // doesn't require lock
	}

	public void flattenWrist() throws InterruptedException {
		outtake.flattenWrist();
	}

	public void releaseClaw1() throws InterruptedException {
		outtake.releaseClaw1(); // doesn't require lock
	}

	public void releaseClaw2() throws InterruptedException {
		outtake.releaseClaw2(); // doesn't require lock
	}

	public void runRigging() {
		outtake.runRigging(); // doesn't require lock
	}

	public void recalibrate() throws InterruptedException {
		if (!intakeLock.tryLock()) throw new IllegalStateException("intake is busy!");
		try {
			if (!outtakeLock.tryLock()) throw new IllegalStateException("outtake is busy!");
			try (var scope = HardwareTaskScope.open()) {
				scope.fork(intake::calibrate);
				scope.fork(outtake::calibrate);

				scope.join();
			} finally {
				outtakeLock.unlock();
			}
		} finally {
			intakeLock.unlock();
		}
	}
}
