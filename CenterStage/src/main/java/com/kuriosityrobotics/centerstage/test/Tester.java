package com.kuriosityrobotics.centerstage.test;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

/**
 * The Tester class is responsible for assisting in testing and debugging code.
 * It provides methods for performing assertions and confirming conditions,
 * as well as displaying information via telemetry.
 */
public class Tester {
	private static final String FAIL_HTML = "<font color=\"red\">fail</font>";
	private static final String PASS_HTML = "<font color=\"green\">pass</font>";

	private final Telemetry telemetry;
	private final Gamepad gamepad;

    /**
	 * Constructs a new Tester object with the given Telemetry and Gamepad.
	 *
	 * @param telemetry the Telemetry object for displaying information
	 * @param gamepad the Gamepad object for user input
	 */
	public Tester(ScheduledExecutorService ses, Telemetry telemetry, Gamepad gamepad) {
		this.telemetry = telemetry;
		this.gamepad = gamepad;

		telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
		telemetry.setAutoClear(false);

		ses.scheduleWithFixedDelay(telemetry::update, 0, 100, TimeUnit.MILLISECONDS);
	}

	private boolean userInput() throws InterruptedException {
		// wait for release
		while (gamepad.a || gamepad.b) {
			Thread.sleep(20);
		}
		boolean a;
		// wait for press
		while (!(a = gamepad.a) && !gamepad.b) {
			Thread.sleep(20);
		}
		return a;
	}

	private void userConfirmation() throws InterruptedException {
		// wait for release
			while (gamepad.a) {
				Thread.sleep(20);
			}

			while (!gamepad.a) {
				Thread.sleep(20);
			}
	}

    /**
	 * Adds a line to the telemetry with a caption and the result of the condition.
	 * It also updates the telemetry display.
	 *
	 * @param caption   the caption for the assertion
	 * @param condition the boolean condition to be evaluated
	 */
	public void automaticAssert(String caption, boolean condition) {
		telemetry.addLine(caption + ": " + (condition ? PASS_HTML : FAIL_HTML));
	}

	/**
	 * Confirms a given condition by displaying it on the telemetry and waiting for user confirmation.
	 *
	 * @param condition the condition to confirm
	 */
	public void confirmThat(String condition) throws InterruptedException {
		var confirmationLine = telemetry.addLine("Confirm that: " + condition);
		var instructionLine = telemetry.addLine("<font color=\"blue\">Press A if true, B if false</font>");

		boolean a = userInput();

		telemetry.removeLine(confirmationLine);
		telemetry.removeLine(instructionLine);

		telemetry.addLine("Confirm that: " + condition + " [" + (a ? PASS_HTML : FAIL_HTML) + "]");
	}

	/**
	 * Adds a line of information to the telemetry and updates the display.
	 *
	 * @param line the line of information to be added to the telemetry
	 */
	public void info(String line) {
		telemetry.addLine(line);
	}

	/**
	 * Adds a continuously-recomputed line of info
	 */
	public void info(String caption, Supplier<String> data) {
		telemetry.addData(caption, data::get);
	}

	/**
	 * Adds a h3 header of info
	 */
	public void header(String line) {
		telemetry.addLine(String.format("<h3>%s</h3>", line));
	}


	/**
	 * Tells the user to perform an action, and waits for them to do so.
	 * @param instruction the instruction to be displayed
	 */
	public void instruct(String instruction) throws InterruptedException {
		telemetry.addLine(instruction);
		telemetry.addLine("<font color=\"blue\">Press A when done</font>");

		userConfirmation();
	}
}
