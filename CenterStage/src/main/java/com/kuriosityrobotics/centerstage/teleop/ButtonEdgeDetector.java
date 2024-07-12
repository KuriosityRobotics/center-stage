package com.kuriosityrobotics.centerstage.teleop;

import com.kuriosityrobotics.centerstage.util.ExceptionRunnable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Collection;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Predicate;

public class ButtonEdgeDetector {
	private final Logger logger = LoggerFactory.getLogger(ButtonEdgeDetector.class);
	private final Gamepad gp;
	private final EnumSet<Button> previouslyPressed = EnumSet.noneOf(Button.class);

	private final EnumMap<Button, Collection<ExceptionRunnable<InterruptedException>>> risingCallbacks = new EnumMap<>(Button.class);
	private final EnumMap<Button, Collection<ExceptionRunnable<InterruptedException>>> fallingCallbacks = new EnumMap<>(Button.class);

	private final ScheduledExecutorService ses;

	public ButtonEdgeDetector(ScheduledExecutorService ses, Gamepad gp) {
		this.ses = ses;
		this.gp = gp;

		ses.scheduleWithFixedDelay(this::publish, 0, 1000 / 50, TimeUnit.MILLISECONDS);
	}

	public void onRising(Button button, ExceptionRunnable<InterruptedException> callback) {
		risingCallbacks.computeIfAbsent(button, __ -> new ConcurrentLinkedQueue<>()).add(callback);
	}

	public void onFalling(Button button, ExceptionRunnable<InterruptedException> callback) {
		fallingCallbacks.computeIfAbsent(button, __ -> new ConcurrentLinkedQueue<>()).add(callback);
	}

	private synchronized void publish() {
		for (var button : Button.values()) {
			var lastValue = previouslyPressed.contains(button);
			var newValue = button.getter.test(gp);

			Collection<ExceptionRunnable<InterruptedException>> callbacks;

			if (!lastValue && newValue) { // rising
				previouslyPressed.add(button);
				callbacks = risingCallbacks.computeIfAbsent(button, __ -> new ConcurrentLinkedQueue<>());
			} else if (lastValue && !newValue) { // falling
				previouslyPressed.remove(button);
				callbacks = fallingCallbacks.computeIfAbsent(button, __ -> new ConcurrentLinkedQueue<>());
			} else { // monotonic
				continue;
			}

			for (var r : callbacks) {
				ses.submit(() -> {
					try {
						r.run();
					} catch (InterruptedException e) {
						logger.info("Interrupted while running callback", e);
						Thread.currentThread().interrupt();
					} catch (RuntimeException e) {
						logger.info("Exception while running callback", e);
						throw e;
					}
				});
			}
		}
	}

	public enum Button {
		A(g -> g.a),
		B(g -> g.b),
		X(g -> g.x),
		Y(g -> g.y),
		LEFT_BUMPER(g -> g.left_bumper),
		RIGHT_BUMPER(g -> g.right_bumper),
		LEFT_TRIGGER(g -> g.left_trigger > 0.1),
		RIGHT_TRIGGER(g -> g.right_trigger > 0.1), // todo: could use joysticks as buttons too
		LEFT_STICK_BUTTON(g -> g.left_stick_button),
		RIGHT_STICK_BUTTON(g -> g.right_stick_button),
		DPAD_UP(g -> g.dpad_up),
		DPAD_DOWN(g -> g.dpad_down),
		DPAD_LEFT(g -> g.dpad_left),
		DPAD_RIGHT(g -> g.dpad_right),
		START(g -> g.start),
		BACK(g -> g.back),
		GUIDE(g -> g.guide),
		LEFT_STICK_UP(g -> g.left_stick_y < -0.1),
		LEFT_STICK_DOWN(g -> g.left_stick_y > 0.1);

		private final Predicate<Gamepad> getter;

		Button(Predicate<Gamepad> getter) {
			this.getter = getter;
		}
	}
}
