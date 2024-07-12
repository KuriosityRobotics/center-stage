package com.kuriosityrobotics.centerstage.util;

import static java.lang.Math.abs;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.RepeatedTest;


public class InstantTest {
    private static final long NS_TO_S = 1_000_000_000;
    private static final long NS_TO_MS = 1_000_000;
    @RepeatedTest(10)
    void testInstantNow() throws InterruptedException{
        Instant begin = Instant.now();
        Thread.sleep(100);
        Instant end = Instant.now();
        assertTrue(end.nanos() - begin.nanos() - NS_TO_S < 5 * NS_TO_MS);
    }

    @RepeatedTest(10)
    void testInstantUntil() throws InterruptedException{
        Instant instant = Instant.now();
        Thread.sleep(100);
        Instant end = Instant.now();
        assertTrue(instant.until(end).toNanos() < NS_TO_S + (5 * NS_TO_MS));
    }

    @RepeatedTest(10)
    void testInstantAdd() throws InterruptedException{
        Instant instant = Instant.now();
        Thread.sleep(100);
        Instant end = Instant.now();
        assertTrue(abs(end.nanos() - instant.add(Duration.ofNanos(100 * NS_TO_MS)).nanos()) < 10 * NS_TO_MS);
    }
}
