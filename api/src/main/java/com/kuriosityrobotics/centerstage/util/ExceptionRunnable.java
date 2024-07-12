package com.kuriosityrobotics.centerstage.util;

@FunctionalInterface
public interface ExceptionRunnable<E extends Throwable> {
	void run() throws E;
}