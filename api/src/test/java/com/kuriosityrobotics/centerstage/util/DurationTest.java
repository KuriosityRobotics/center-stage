package com.kuriosityrobotics.centerstage.util;

import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;

// class based on assumption that Instant works
class DurationTest {
    // if this one fails then there is no saving us
    @Test
    public void initializerTest(){
        Instant i = Instant.now();
        Duration d = Duration.ofNanos(i.nanos());
        assert(d.toNanos() == i.nanos());
    }

    @RepeatedTest(10)
    public void testToMillis(){
        Instant i = Instant.now();
        Duration d = Duration.ofNanos(i.nanos());
        assert(i.toEpochMilli() == d.toMillis());
    }
}