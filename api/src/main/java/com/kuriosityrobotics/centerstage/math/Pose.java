package com.kuriosityrobotics.centerstage.math;

import static java.lang.Math.toDegrees;

import java.io.Serializable;
import java.util.Locale;
import java.util.Objects;

/**
 * A representation of pose in free space, composed of position and orientation.
 */
public class Pose extends Point implements Serializable {
   private final double orientation;

   /**
	* A representation of pose in free space, composed of position and orientation.
	*
	* @param orientation the orientation of the pose, in radians
	*/
   public Pose(double x, double y, double orientation) {
	  super(x, y);
	  this.orientation = orientation;
   }

   public Pose(Point point, double orientation){
       super(point.x(), point.y());
       this.orientation = orientation;
   }

   public static Pose ofMeters(double x, double y, double orientation) {
	   return new Pose( x, y, orientation);
   }

    public static Pose of(double x, double y, double theta) {
	   		return new Pose(x, y, theta);
    }

    public double orientation() {
	  return orientation;
   }

   public Pose add(Twist twist) {
	  return new Pose(
			  this.x() + twist.x(),
			  this.y() + twist.y(),
			  this.orientation() + twist.angular());
   }

   public Pose subtract(Twist twist) {
	   return new Pose(
		   this.x() - twist.x(),
		   this.y() - twist.y(),
		   this.orientation() - twist.angular());
   }

   public Pose metres() {
	   return new Pose(x(), y() , orientation);
   }

   private static final Pose ZERO = new Pose(0, 0, 0);
   public static Pose zero() {
	  return ZERO;
   }

   public Pose withAngle(double angle) {
	   return new Pose(x(), y(), angle); // wither pattern??
   }

	public Pose toFTCSystem() {
		double x = -y() + (144. / 2.);
		double y = x() - (144. / 2.);
		double heading = MathUtil.angleWrap(Math.PI - orientation);
		return new Pose(x, y, heading);
	}

	@Override
   public boolean equals(Object obj) {
	  if (obj == this) return true;
	  if (obj == null || obj.getClass() != this.getClass()) return false;
	  var that = (Pose) obj;
	  return this.x() == that.x() &&
			  this.y() == that.y() &&
			  this.orientation == that.orientation;
   }

   @Override
   public int hashCode() {
	  return Objects.hash(x(), y(), orientation);
   }

   @Override
   public String toString() {
	   return String.format(Locale.getDefault(), "[%.5f, %.5f, %.5f]", x(), y(), toDegrees(orientation));
   }

	public double[] toArray() {
		return new double[]{x(), y(), orientation};
	}

	/**
	 * Mirrors the Pose across the X-axis,
	 * negating the Y and Theta position and angle values.
	 */
	public Pose mirror() {
		return new Pose(x(), -y(), -orientation);
	}
}
