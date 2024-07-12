package com.kuriosityrobotics.centerstage.concurrent;

import com.kuriosityrobotics.centerstage.util.Duration;
import com.kuriosityrobotics.centerstage.util.Instant;

import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

public class FairRateLimiter {
	private final ReentrantLock lock = new ReentrantLock();
	private final ConcurrentLinkedDeque<Condition> waiters = new ConcurrentLinkedDeque<>();
	private Instant nextFree;
	private final Duration minimumDelay;

	public FairRateLimiter(Duration minimumDelay) {
		this.minimumDelay = minimumDelay;
		this.nextFree = Instant.now();
	}

	// guarded by lock
	private void pollAndSignalNext() {
		waiters.pop();
		Condition next = waiters.peek();
		if (next != null)
			next.signal();
	}

	// guarded by lock
	private void scheduleNextTime(Instant now) {
		this.nextFree = (now.add(minimumDelay));
	}

	public void use() throws InterruptedException {
		Condition condition = lock.newCondition();

		lock.lock();
		try {
			waiters.addLast(condition);

			while (waiters.peekFirst() != condition)
				condition.await();

			Instant now;
			while ((now = Instant.now()).isBefore(nextFree))
				condition.await(nextFree.since(now).toMillis(), TimeUnit.MILLISECONDS);

			pollAndSignalNext();
			scheduleNextTime(now);
		} finally {
			lock.unlock();
		}
	}

	public boolean tryUse() {
		lock.lock();
		try {
			Instant now = Instant.now();
			if (waiters.isEmpty() && now.isAfter(nextFree)) {
				scheduleNextTime(now);
				return true;
			} else {
				return false;
			}
		} finally {
			lock.unlock();
		}
	}

	public boolean tryUse(Duration timeout) throws InterruptedException {
		Condition condition = lock.newCondition();
		Instant end = Instant.now().add(timeout);

		lock.lock();
		try {
			waiters.addLast(condition);

			for (; ; ) {
				Instant now = Instant.now();

				if (waiters.peekFirst() == condition && now.isAfter(nextFree)) {
					pollAndSignalNext();
					scheduleNextTime(now);
					return true;
				}

				if (now.isAfter(end)) {
					waiters.remove(condition);
					return false;
				}

				Duration timeRemaining = end.since(now);
				condition.await(timeRemaining.toMillis(), TimeUnit.MILLISECONDS);
			}
		} finally {
			lock.unlock();
		}
	}
}
