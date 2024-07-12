package com.kuriosityrobotics.centerstage.concurrent;

import org.junit.jupiter.api.Test;

import java.time.Duration;
import java.util.concurrent.CountDownLatch;

import static org.junit.jupiter.api.Assertions.*;

public class PreemptibleLockTest {

	@Test
	void testLockWhenUnlocked() {
		PreemptibleLock lock = new PreemptibleLock();
		assertFalse(lock.isLocked());
		lock.lock();
		assertTrue(lock.isLocked());
	}

	@Test
	void testLockWhenLockedByAnotherThread() throws InterruptedException {
		PreemptibleLock lock = new PreemptibleLock();

		Thread t1 = new Thread(() -> {
			lock.lock();
			try {
				//Hold lock for a while
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// Expected interruption
			} finally {
				lock.unlock();
			}
		});

		Thread t2 = new Thread(() -> {
			lock.lock();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				throw new RuntimeException(e);
			}
			lock.unlock();
		});

		t1.start();
		try {
			Thread.sleep(200); // Let t1 acquire the lock
		} catch (InterruptedException e) {
			// ignore
		}

		t2.start();

		assertTimeout(Duration.ofMillis(100), () -> t1.join());
		assertTrue(t2.isAlive());
		assertTrue(lock.isLocked());

		t2.join();

		assertFalse(t1.isAlive());
		assertFalse(lock.isLocked());
	}

	@Test
	void testLockUnderHighContention() throws InterruptedException {
		final int threadCount = 1000;
		final PreemptibleLock lock = new PreemptibleLock();
		final CountDownLatch latch = new CountDownLatch(threadCount);
		class Holder {
			int counter = 0;
		}
		var holder = new Holder();

		// Spawn a bunch of threads that try to lock and increment counter
		for (int i = 0; i < threadCount; i++) {
			new Thread(() -> {
				latch.countDown();
				try {
					latch.await(); // Wait for all threads to be ready
					lock.lock();
					try {
						holder.counter++; // Protected by lock
					} finally {
						lock.unlock();
					}
				} catch (InterruptedException ignored) {
				}
			}).start();
		}

		// Wait for all threads to finish
		while (latch.getCount() > 0) {
			Thread.sleep(100); // Sleep a while
		}
		Thread.sleep(1000); // Allow threads to finish

		// Check for race conditions or deadlocks
		assertEquals(threadCount, holder.counter);
		assertFalse(lock.isLocked());
	}

}