package com.kuriosityrobotics.centerstage.concurrent;

import com.kuriosityrobotics.centerstage.util.Duration;
import com.kuriosityrobotics.centerstage.util.Instant;

import java.util.concurrent.atomic.AtomicReference;

/**
 * An unfair (racing) rate limiter that can be used
 * to limit the rate at which a certain action is performed.
 * <p>
 * Uses do not need to be freed afterwards.
 * <p>
 * This class is thread-safe.
 */
public class RateLimiter {
	private final AtomicReference<Instant> nextFree;
	private final Duration minimumDelay;

	public RateLimiter(Duration minimumDelay) {
		this.minimumDelay = minimumDelay;
		nextFree = new AtomicReference<>(Instant.now());
	}

	public void use() throws InterruptedException {
		for (; ; ) {
			Instant now = Instant.now();
			Instant nextFreeTime = nextFree.get();

			if (now.isBefore(nextFreeTime))
				Thread.sleep(nextFreeTime.since(now).toMillis());

			if (nextFree.compareAndSet(nextFreeTime, now.add(minimumDelay)))
				return;
		}
	}

	public boolean tryUse() {
		for (; ; ) {
			Instant now = Instant.now();
			Instant nextFreeTime = nextFree.get();

			if (now.isBefore(nextFreeTime))
				return false;

			if (nextFree.compareAndSet(nextFreeTime, now.add(minimumDelay)))
				return true;
		}
	}

	public boolean tryUse(Duration timeout) throws InterruptedException {
		Instant end = Instant.now().add(timeout);
		for (; ; ) {
			Instant now = Instant.now();
			Instant nextFreeTime = nextFree.get();

			if (nextFreeTime.isAfter(end))
				return false;

			if (now.isBefore(nextFreeTime))
				Thread.sleep(nextFreeTime.since(now).toMillis());

			if (nextFree.compareAndSet(nextFreeTime, now.add(minimumDelay)))
				return true;
		}
	}
}
