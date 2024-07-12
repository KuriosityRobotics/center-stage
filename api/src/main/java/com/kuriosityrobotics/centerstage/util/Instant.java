package com.kuriosityrobotics.centerstage.util;

public final class Instant implements Comparable<Instant> {
    private static final long nanoTimeEpochOffset = System.nanoTime() - System.currentTimeMillis() * 1_000_000L;
	private static final long robotStartTime = System.currentTimeMillis() * 1_000_000L;
	private static final Instant start = Instant.now();

    private final long nanos;

    public static Instant now() {
        return new Instant(System.nanoTime() - nanoTimeEpochOffset);
    }

	public static Instant ofEpochMillis(long millis) {
		return new Instant(millis * 1_000_000);
	}

    private Instant(long time) {
        nanos = time;
    }

	public static Instant createInstant(long time) {
		return new Instant(time);
	}

	public Duration sinceStart() {
		return since(start);
	}

    public boolean isBefore(Instant instant) {
        return nanos < instant.nanos;
    }

    public boolean isAfter(Instant instant) {
        return nanos > instant.nanos;
    }

    public Duration since(Instant time) {
        return Duration.ofNanos(nanos - time.nanos);
    }

    public Duration until(Instant time) {
        return since(time).negated();
    }

	public long nanos() {
		return nanos;
	}

    public Instant add(Duration duration) {
        return new Instant(nanos + duration.toNanos());
    }

    public Instant minus(Duration duration) {
        return new Instant(nanos - duration.toNanos());
    }

    public long toEpochMilli() { // needed to fix NetworkMessage but may need additional fixing
        return nanos / NANOS_PER_MILLI;
    }

	public double toEpochMilliD() { // needed to fix NetworkMessage but may need additional fixing
		return (double)nanos / NANOS_PER_MILLI;
	}

    public boolean equals(Instant other) {
        return this.nanos == other.nanos;
    }

    @SuppressWarnings("DefaultLocale")
    public String toString() {
        var millis = (nanos - robotStartTime) / NANOS_PER_MILLI;
        var minutes = millis / (MILLIS_PER_SECOND * SECONDS_PER_MINUTE);
        var seconds = (millis / MILLIS_PER_SECOND) % SECONDS_PER_MINUTE;
        var millisOfSecond = millis % MILLIS_PER_SECOND;

        return String.format("%d:%d.%3d", minutes, seconds, millisOfSecond);
    }


    /*
     * COMPARABLE METHODS
     */
    @Override
    public int compareTo(Instant other) {
        if (this.equals(other)) return 0;
        if (other.nanos > this.nanos) return -1;
        return 1;
    }

    private static final int HOURS_PER_DAY = 24;
    /**
     * Minutes per hour.
     */
    private static final int MINUTES_PER_HOUR = 60;
    /**
     * Minutes per day.
     */
    static final int MINUTES_PER_DAY = MINUTES_PER_HOUR * HOURS_PER_DAY;
    /**
     * Seconds per minute.
     */
    private static final int SECONDS_PER_MINUTE = 60;
    /**
     * Seconds per hour.
     */
    private static final int SECONDS_PER_HOUR = SECONDS_PER_MINUTE * MINUTES_PER_HOUR;
    /**
     * Seconds per day.
     */
    private static final int SECONDS_PER_DAY = SECONDS_PER_HOUR * HOURS_PER_DAY;
    /**
     * Milliseconds per second.
     */
    private static final long MILLIS_PER_SECOND = 1000L;
    /**
     * Milliseconds per day.
     */
    static final long MILLIS_PER_DAY = MILLIS_PER_SECOND * SECONDS_PER_DAY;
    /**
     * Microseconds per second.
     */
    private static final long MICROS_PER_SECOND = 1000_000L;
    /**
     * Microseconds per day.
     */
    static final long MICROS_PER_DAY = MICROS_PER_SECOND * SECONDS_PER_DAY;
    /**
     * Nanos per millisecond.
     */
    private static final long NANOS_PER_MILLI = 1000_000L;
    /**
     * Nanos per second.
     */
    private static final long NANOS_PER_SECOND = 1000_000_000L;
    /**
     * Nanos per minute.
     */
    private static final long NANOS_PER_MINUTE = NANOS_PER_SECOND * SECONDS_PER_MINUTE;
    /**
     * Nanos per hour.
     */
    private static final long NANOS_PER_HOUR = NANOS_PER_MINUTE * MINUTES_PER_HOUR;
    /**
     * Nanos per day.
     */
    static final long NANOS_PER_DAY = NANOS_PER_HOUR * HOURS_PER_DAY;
}
