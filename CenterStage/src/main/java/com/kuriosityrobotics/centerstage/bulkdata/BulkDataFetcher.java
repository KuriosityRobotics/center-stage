package com.kuriosityrobotics.centerstage.bulkdata;

import com.kuriosityrobotics.centerstage.hardware.LynxHub;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.Collection;
import java.util.Set;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.Phaser;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

import static com.kuriosityrobotics.centerstage.hardware.LynxHub.CONTROL_HUB;
import static com.kuriosityrobotics.centerstage.hardware.LynxHub.EXPANSION_HUB;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * The BulkDataFetcher class is responsible for fetching bulk data from the Control Hub and Expansion Hub,
 * converting it to RevHubBulkData objects, and dispatching it to registered listeners.
 * <p>
 * The bulk data is fetched at a fixed rate specified by the {@link ScheduledExecutorService} passed in the constructor.
 * The fetched data is then converted using the dispatchBulkData method and dispatched through different topics.
 */
public class BulkDataFetcher {
	private static final Logger logger = LoggerFactory.getLogger(BulkDataFetcher.class);
	private final Collection<Consumer<RevHubBulkData>> controlHubListeners = new LinkedBlockingQueue<>();
	private final Collection<Consumer<RevHubBulkData>> expansionHubListeners = new LinkedBlockingQueue<>();

	private final LynxModule controlHub;
	private final LynxModule expansionHub;

	public BulkDataFetcher(ScheduledExecutorService ses, LynxModule controlHub, LynxModule expansionHub) {
		this.controlHub = controlHub;
		this.expansionHub = expansionHub;

		ses.scheduleWithFixedDelay(this::updateData, 0, 1000 / 50, TimeUnit.MILLISECONDS);
	}

	/**
	 * Adds a listener to receive bulk data from the Control Hub.
	 *
	 * @param listener The listener to add.
	 */
	public void addControlHubListener(Consumer<RevHubBulkData> listener) {
		controlHubListeners.add(listener);
	}

	/**
	 * Adds a listener to receive bulk data from the Expansion Hub.
	 *
	 * @param listener The listener to add.
	 */
	public void addExpansionHubListener(Consumer<RevHubBulkData> listener) {
		expansionHubListeners.add(listener);
	}

	/**
	 * Returns a {@link BulkDataNotifier} for the specified LynxHubs.
	 *
	 * @param lynxHub The LynxHubs to notify.
	 * @return The BulkDataNotifier instance.
	 * @throws IllegalArgumentException if no LynxHubs are specified.
	 */
	public BulkDataNotifier notifier(LynxHub... lynxHub) {
		if (lynxHub.length == 0)
			throw new IllegalArgumentException("Must specify at least one hub");

		var notifier = new BulkDataNotifier();

		var set = Set.of(lynxHub);
		if (set.contains(CONTROL_HUB))
			addControlHubListener(data -> notifier.sendNotification());

		if (set.contains(EXPANSION_HUB))
			addExpansionHubListener(data -> notifier.sendNotification());

		return notifier;
	}

	/**
	 * Updates the data by dispatching the bulk data from the Control Hub and Expansion Hub.
	 */
	private void updateData() {
		dispatchBulkData(CONTROL_HUB, controlHub.getBulkData());
		dispatchBulkData(EXPANSION_HUB, expansionHub.getBulkData());
	}

	/**
	 * Converts the BulkData to a {@link RevHubBulkData}.
	 * <p>
	 * Dispatches the bulk data through many topics.
	 *
	 * @param data The bulk data to convert.
	 */
	private void dispatchBulkData(LynxHub hub, LynxModule.BulkData data) {
		var result = new RevHubBulkData(hub);

		for (int i = 0; i < result.encoders.length; i++) {
			var position = data.getMotorCurrentPosition(i);
			result.encoders[i] = position;
		}


		for (int i = 0; i < result.velocities.length; i++) {
			var correctedVelocity = correctOverflow(data.getMotorVelocity(i));
			result.velocities[i] = correctedVelocity;
		}

		for (int i = 0; i < result.analogInputs.length; i++) {
			var voltage = data.getAnalogInputVoltage(i);
			result.analogInputs[i] = voltage;
		}

		for (int i = 0; i < result.digitalInputs.length; i++) {
			var state = data.getDigitalChannelState(i);
			result.digitalInputs[i] = state;
		}

		switch (hub) {
			case CONTROL_HUB:
				controlHubListeners.forEach(listener -> listener.accept(result));
				break;
			case EXPANSION_HUB:
				expansionHubListeners.forEach(listener -> listener.accept(result));
				break;
		}
	}

	private int correctOverflow(int input) {
		if (input % 20 == 0) {
			return input;
		} else if ((input + (1 << 16)) % 20 == 0) {
			return input + (1 << 16);
		} else if ((input - (1 << 16)) % 20 == 0) {
			return input - (1 << 16);
		} else if ((input + 2 * (1 << 16)) % 20 == 0) {
			return input + 2 * (1 << 16);
		} else if ((input - 2 * (1 << 16)) % 20 == 0) {
			return input - 2 * (1 << 16);
		} else {
			logger.warn("Could not figure out how to prevent overflow and cast vel to multiple of 20");
			return input; // this edge case is not possible when using a throughbore encoder, where the reported velocity is always a multiple of 20 in some form
		}
	}

	public static class BulkDataNotifier {
		private final Phaser barrier = new Phaser(1);

		/**
		 * A barrier for notifications which notifies waiters
		 * when a bulk data update arrives via
		 * <code>barrier.arrive()</code>.
		 * This code causes the phase to advance,
		 * as the only (one) registered party arrives.
		 */
		private void sendNotification() {
			barrier.arrive();
		}

		/**
		 * Awaits a bulk data notification.
		 * This code is thread-safe because:
		 * <p>
		 * if the phase advances after polling <code>barrier.getPhase()</code>, the method returns
		 * immediately, because an advance already happened.
		 * <p>
		 * if the phase is equivalent to <code>barrier.getPhase()</code>, the method waits
		 * as usual for the next advancement.
		 */
		public void await() throws InterruptedException {
			barrier.awaitAdvanceInterruptibly(barrier.getPhase());
		}
	}
}
