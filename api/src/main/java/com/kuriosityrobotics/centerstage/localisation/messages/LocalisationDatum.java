package com.kuriosityrobotics.centerstage.localisation.messages;

import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;
import com.kuriosityrobotics.centerstage.util.Instant;

import java.io.Serializable;
import java.util.Objects;

import static java.lang.Math.toDegrees;

/**
 * This represents an timed estimate of a position and velocity in free space.
 * Contains a timestamp {@link Instant}, a {@link Pose} and a {@link Twist}.
 * */
public final class LocalisationDatum implements Serializable {

	private final Pose pose;
	private final Twist twist;

	public LocalisationDatum(Pose pose, Twist twist) {
		this.pose = pose;
		this.twist = twist;
	}

	public static LocalisationDatum of(Pose pose, Twist twist) {
	   return new LocalisationDatum(pose, twist);
	}

	public static LocalisationDatum zero() {
		return of(Pose.zero(), Twist.zero());
	}

	public Pose pose() {
		return pose;
	}

	public Twist twist() {
		return twist;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == this) return true;
		if (obj == null || obj.getClass() != this.getClass()) return false;
		var that = (LocalisationDatum) obj;
		return Objects.equals(this.pose, that.pose)
				&& Objects.equals(this.twist, that.twist);
	}

	@Override
	public int hashCode() {
		return Objects.hash( pose, twist);
	}

	@Override
	public String toString() {
		// Nicely format the pose and velocity, but omit the stamp
		return String.format(
				"%02.2f m, %02.2f m, %03.0f deg\n@ %02.2f m/s, %02.2f m/s, %03.1f deg/s",
				pose.x(),
				pose.y(),
				toDegrees(pose.orientation()),
				twist.x(),
				twist.y(),
				toDegrees(twist.angular()));
	}
}
