package com.kuriosityrobotics.centerstage.concurrent;

import static java.util.Objects.requireNonNull;

class Util {
	public static String toIdentityString(Object o) {
		requireNonNull(o);
		return o.getClass().getName() + "@" + Integer.toHexString(System.identityHashCode(o));
	}
}