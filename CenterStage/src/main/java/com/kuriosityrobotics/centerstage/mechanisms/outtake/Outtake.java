package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import androidx.annotation.GuardedBy;
import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;

import java.util.concurrent.locks.ReentrantLock;

//import static com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeArm.OuttakeArmPosition.DOCKING;
import static com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeArm.OuttakeArmPosition.PIXEL_DROP;
import static com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeArm.OuttakeArmPosition.TRANSFER;
import static com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeExtensionSlides.OuttakeSlidePosition.*;
import static com.kuriosityrobotics.centerstage.mechanisms.outtake.OuttakeExtensionSlides.SlidesJammed;

public class Outtake {
	private final ReentrantLock lock = new ReentrantLock();

	@GuardedBy("lock")
	private final OuttakeExtensionSlides slides;
	@GuardedBy("lock")
	private final OuttakeArm arm;
	@GuardedBy("lock")
	private final OuttakeWrist wrist;
	@GuardedBy("lock")
	private final OuttakeClaw claw1; // "upper"?
	@GuardedBy("lock")
	private final OuttakeClaw claw2; // "lower"?

	@GuardedBy("lock")
	private OuttakeExtensionSlides.OuttakeSlidePosition slidePosition = RETRACTED;
	@GuardedBy("lock")
	private OuttakeArm.OuttakeArmPosition armPosition = TRANSFER;
	@GuardedBy("lock")
	private boolean canUseWrist = false;

	public Outtake(OuttakeExtensionSlides slides, OuttakeArm arm, OuttakeWrist wrist, OuttakeClaw claw1, OuttakeClaw claw2) {
		this.slides = slides;
		this.arm = arm;
		this.wrist = wrist;
		this.claw1 = claw1;
		this.claw2 = claw2;
	}

	public void calibrate() throws InterruptedException {
		slides.calibrate();
	}

	@GuardedBy("lock")
	private void toArmPosition0(OuttakeArm.OuttakeArmPosition position) throws InterruptedException {
		arm.goToPosition(position);
		armPosition = position;
	}

	public void toPurplePreload() throws InterruptedException {
		if (!lock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			if (armPosition == TRANSFER) {
				toDepositHeight0(OVER_PASS);
			}

			toDepositHeight0(RETRACTED);
			toArmPosition0(PIXEL_DROP);
		} catch (SlidesJammed e) {
			throw new RuntimeException("could not move to deposit", e); // TODO: in auto, maybe we want more sophisticated handling of this?
		} finally {
			lock.unlock();
		}
	}

	public void depositPurple() throws InterruptedException {
		releaseClaw1();
	}

	@GuardedBy("lock")
	private void toDepositHeight0(OuttakeExtensionSlides.OuttakeSlidePosition position) throws InterruptedException, SlidesJammed {
//		if (armResting && position == RETRACTED)
//			throw new IllegalStateException("outtake cannot deposit without lifting slides!");
		slides.goToPositions(position);
		slidePosition = position;

		toArmPosition0(OuttakeArm.OuttakeArmPosition.DEPOSIT); // shouldn't take time if already there
		canUseWrist = true;
	}

	/**
	 * Adjusts the height of the outtake arm, to prepare for deposit,
	 * and opens the arm if not already opened.
	 * <p>
	 * This method should be used in order to either to open outtake for depositing,
	 * or to adjust the height if already opened.
	 * <p>
	 * This code is thread-safe.
	 */
	public void toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition position) throws InterruptedException {
		if (!lock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			if (armPosition == TRANSFER) {
				toArmPosition0(TRANSFER);
				if (position.requiresOverPass())
					toDepositHeight0(OVER_PASS); // avoid collision with the robot wall first
			}

			toDepositHeight0(position);
		} catch (SlidesJammed e) {
			throw new RuntimeException("could not move to deposit", e); // TODO: in auto, maybe we want more sophisticated handling of this?
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Releases the pixels from the outtake, and then
	 * collapses the outtake to the resting position.
	 * <p>
	 * This code is thread-safe.
	 */
	public void releaseAndCollapse() throws InterruptedException {
		if (!lock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try (var scope = HardwareTaskScope.open(SlidesJammed.class)) {
			canUseWrist = false;
			if (armPosition == TRANSFER)
				throw new IllegalStateException("arm is already resting");

			scope.fork(claw1::unlatch);
			scope.fork(claw2::unlatch);
			scope.join();

			Thread.sleep(750);

			scope.fork(this::resetArm0);
			scope.fork(this::resetSlides0);

			scope.join();
		} catch (SlidesJammed e) {
            throw new RuntimeException(e);
        } finally {
			lock.unlock();
		}
	}

	@GuardedBy("lock")
	private void resetArm0() throws InterruptedException {
		try (var scope = HardwareTaskScope.open()) {
			scope.fork(wrist::toNeutral);
//			scope.fork(claw1::latch); // not strictly necessary
//			scope.fork(claw2::latch);
			scope.fork(arm::goToTransfer);

			scope.join();
			armPosition = TRANSFER; // depends on successful completion of all tasks
		}
	}

	@GuardedBy("lock")
	private void resetSlides0() throws InterruptedException, SlidesJammed {
		slides.goToPositions(RETRACTED);
		slidePosition = RETRACTED;
	}

	/**
	 * Raises the outtake arm to the transfer position.
	 * This method should be used in the transfer sequence,
	 */
	public void transferArm() throws InterruptedException {
		if (!lock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			canUseWrist = false;
			if (armPosition != TRANSFER || slidePosition != RETRACTED)
				throw new IllegalStateException("arm must be resting");
			toArmPosition0(TRANSFER);
		} finally {
			lock.unlock();
		}
	}

	public void latchClaws() throws InterruptedException {
		try (var scope = HardwareTaskScope.open()) {
//			try {
//				slides.goToPositions(LOW_ROW);
//			} catch (SlidesJammed e) {
//				throw new RuntimeException(e); // whatever
//			}
			scope.fork(claw1::latch);
			scope.fork(claw2::latch);
			scope.join();
		}
	}

	public void unlatchClaws() throws InterruptedException {
		try (var scope = HardwareTaskScope.open()) {
			scope.fork(claw1::unlatch);
			scope.fork(claw2::unlatch);
			scope.join();
		}
	}

	public void incrementHeight() throws InterruptedException {
		if (slidePosition == MAX_HEIGHT) throw new IllegalStateException("already at max height!");
		toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.values()[slidePosition.ordinal() + 1]);
	}

	public void decrementHeight() throws InterruptedException {
		if (slidePosition == RETRACTED) throw new IllegalStateException("already at min height!");
		toDepositHeight(OuttakeExtensionSlides.OuttakeSlidePosition.values()[slidePosition.ordinal() - 1]);
	}

	public void incrementWrist() throws InterruptedException {
		if (!wristIsSafe()) throw new IllegalStateException("wrist is not safe to move");
		wrist.incrementAngle();
	}

	public void decrementWrist() throws InterruptedException {
		if (!wristIsSafe()) throw new IllegalStateException("wrist is not safe to move");
		wrist.decrementAngle();
	}

	public void flattenWrist() throws InterruptedException {
		if (!wristIsSafe()) throw new IllegalStateException("wrist is not safe to move");
		wrist.toHorizontal();
	}

	public boolean armResting() {
		return armPosition == TRANSFER;
	}

	public boolean wristIsSafe() {
		return canUseWrist;
	}

	public void releaseClaw1() throws InterruptedException {
		if (!wristIsSafe()) throw new IllegalStateException("claws aren't safe to move");
		claw1.unlatch();
	}

	public void releaseClaw2() throws InterruptedException {
		if (!wristIsSafe()) throw new IllegalStateException("claws aren't safe to move");
		claw2.unlatch();
	}

	public OuttakeExtensionSlides.OuttakeSlidePosition slidePosition() {
		return slidePosition;
	}

	public void runRigging() {
		if (!lock.tryLock()) throw new IllegalStateException("outtake is busy!");
		try {
			slides.runRigging();
		} finally {
			lock.unlock();
		}
	}
}