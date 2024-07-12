package com.kuriosityrobotics.centerstage.util;

@FunctionalInterface
public interface ExceptionSupplier<T, E extends Throwable> {
	T run() throws E;
}