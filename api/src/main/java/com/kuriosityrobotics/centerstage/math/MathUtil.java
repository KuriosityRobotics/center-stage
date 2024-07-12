package com.kuriosityrobotics.centerstage.math;

public class MathUtil {
    public static final double EPSILON = 1E-8;

    public static int clamp(int value, int min, int max){
        if(value < min) return min;
        if(value > max) return max;
        return value;
    }

	public static double clamp(double value, double min, double max){
		if(value < min) return min;
		if(value > max) return max;
		return value;
	}

    public static double average(double val1, double val2){
        return (val1 + val2) / 2;
    }

    public static boolean doublesEqual(double a, double b){
        return Math.abs(a-b) < EPSILON;
    }

    /**
     * Wraps angle (in radians) to a value from -pi to pi
     *
     * @param angle Angle to be wrapped
     * @return The wrapped angle, between -pi and pi
     */
    public static double angleWrap(double angle) {
        return angleWrap(angle, 0);
    }

    /**
     * Wraps an angle (in radians) to a value within pi of centerOfWrap.
     *
     * @param angle        The angle to be wrapped
     * @param centerOfWrap The center of the boundary in which the angle can be wrapped to.
     * @return The wrapped angle, which will lie within pi of the centerOfWrap.
     */
    public static double angleWrap(double angle, double centerOfWrap) {
        angle -= centerOfWrap;

        double newAng = angle - ((2 * Math.PI) * Math.round(angle / (2 * Math.PI)));

        return newAng + centerOfWrap;
    }

	public static boolean inRange(double value, double min, double max) {
		return value >= min && value <= max;
	}
}
