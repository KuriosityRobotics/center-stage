package com.kuriosityrobotics.centerstage.util;

public final class Duration implements Comparable<Duration> {
	private final long nanos;
	private static final long ONE_MILLION = 1_000_000;
	private static final long ONE_BILLION = 1_000 * ONE_MILLION;

	private Duration(long time) {
		nanos = time;
	}

	public static Duration ofNanos(long nanos) {
		return new Duration(nanos);
	}

	public static Duration ofMillis(long millis) {
		return new Duration(millis * ONE_MILLION);
	}

	public static Duration ofSeconds(double seconds) {
		return ofNanos((long)(seconds * ONE_BILLION));
	}

	public static Duration between(Instant now, Instant deadline) {
		return new Duration(deadline.nanos() - now.nanos());
	}

	public Duration negated() {
		return new Duration(-nanos);
	}

	public long toNanos() {
		return nanos;
	}

	public long toMillis() {
		return (nanos - (nanos % ONE_MILLION)) / ONE_MILLION;
	}

	public double toSeconds() {
		return nanos / (double) ONE_BILLION;
	}

	public boolean isGreaterThan(Duration duration) {
		return nanos > duration.nanos;
	}

	public boolean isLessThan(Duration duration) {
		return nanos < duration.nanos;
	}

	public boolean equals(Duration other) {
		return this.nanos == other.nanos;
	}

	@Override
	public int compareTo(Duration other) {
		if (this.equals(other)) return 0;
		if (other.nanos > this.nanos) return -1;
		return 1;
	}
}