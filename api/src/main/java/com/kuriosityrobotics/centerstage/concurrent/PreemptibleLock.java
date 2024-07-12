package com.kuriosityrobotics.centerstage.concurrent;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A lock that can be preempted by another thread.
 * If a lock is 'preempted', the holder of the lock will be interrupted.
 */
public class PreemptibleLock extends ReentrantLock {
	public PreemptibleLock() {
		super();
	}

	public PreemptibleLock(boolean fair) {
		super(fair);
	}


	private final ReentrantLock acquisitionLock = new ReentrantLock();
	private final Condition acquisitionCondition = acquisitionLock.newCondition();

	@Override
	public void unlock() {
		acquisitionLock.lock();
		try {
			super.unlock();
			acquisitionCondition.signalAll();
		} finally {
			acquisitionLock.unlock();
		}
	}

	@Override
	public void lock() {
		acquisitionLock.lock();
		try {
			if (!isHeldByCurrentThread() && isLocked()) {
				getOwner().interrupt();
			}
			
			while (!super.tryLock()) {
                acquisitionCondition.awaitUninterruptibly();
            }
		} finally {
			acquisitionLock.unlock();
		}
	}

	@Override
	public void lockInterruptibly() throws InterruptedException {
		acquisitionLock.lockInterruptibly();
		try {
			if (!isHeldByCurrentThread() && isLocked()) {
				getOwner().interrupt();
			}
			
			while (!super.tryLock()) {
				acquisitionCondition.await();
			}
		} finally {
			acquisitionLock.unlock();
		}
	}
}