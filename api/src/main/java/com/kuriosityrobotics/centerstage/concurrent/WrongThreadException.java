package com.kuriosityrobotics.centerstage.concurrent;

public class WrongThreadException extends RuntimeException {
	public WrongThreadException() {
		super();
	}

	public WrongThreadException(String message) {
		super(message);
	}
}