package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import com.kuriosityrobotics.centerstage.hardware.ServoControl;
import com.kuriosityrobotics.centerstage.concurrent.HardwareTaskScope;
import com.kuriosityrobotics.centerstage.test.Tester;

import static java.lang.Math.toRadians;

/**
 * The OuttakeArm is powered by two servos bolted together.
 */
public class OuttakeArm {
	private final ServoControl leveler;
	private final ServoControl angler;

	public enum OuttakeArmPosition {
		TRANSFER(toRadians(-60), toRadians(-50)),
		DEPOSIT(toRadians(180), toRadians(130)),
		PIXEL_DROP(toRadians(210), toRadians(210));

		private final double angle;
		private final double slope;

		OuttakeArmPosition(double angle, double slope) {
			this.angle = angle;
			this.slope = slope;
		}
	}

	public OuttakeArm(OuttakeArmLeveler leveler, OuttakeArmAngler angler) throws InterruptedException {
		this.leveler = leveler;
		this.angler = angler;

		goToPosition(OuttakeArmPosition.TRANSFER);
	}

	private void goToAngle(OuttakeArmPosition position) throws InterruptedException {
		// todo: there's no real way to correct out of bounds, because the two servos are linked
		if (
			!leveler.isInBounds(position.slope) ||
			!angler.isInBounds(position.angle)
		) throw new IllegalArgumentException("Angle out of bounds: " + position);

		try (var scope = HardwareTaskScope.open()) {
			scope.fork(() -> leveler.goToAngle(position.slope));
			scope.fork(() -> angler.goToAngle(position.angle));

			scope.join();
		}
	}

	public void goToPosition(OuttakeArmPosition position) throws InterruptedException {
		goToAngle(position);
	}

	// actually useful methods
	public void goToDeposit() throws InterruptedException {
		goToPosition(OuttakeArmPosition.DEPOSIT);
	}

	public void goToTransfer() throws InterruptedException {
		goToPosition(OuttakeArmPosition.TRANSFER);
	}

//	public void goToDocking() throws InterruptedException {
//		goToPosition(OuttakeArmPosition.DOCKING);
//	}

	public void testRoutine(Tester tester) throws InterruptedException {
		tester.header("[Outtake arm]");

		goToTransfer();
		tester.confirmThat("arm is in resting position");

		goToDeposit();
		tester.confirmThat("arm is in deposit position");
	}
}
