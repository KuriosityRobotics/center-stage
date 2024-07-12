package com.kuriosityrobotics.centerstage.mpc;


/**
 * This class is based on the following struct:
	struct drive_parameters {
		double motor_constant_e;
		double motor_constant_t;
		double armature_resistance;

		double robot_mass;
		double robot_moment;
		double wheel_moment;
		double roller_moment;

		double fl_wheel_friction;
		double fr_wheel_friction;
		double bl_wheel_friction;
		double br_wheel_friction;

		double fl_roller_friction;
		double fr_roller_friction;
		double bl_roller_friction;
		double br_roller_friction;

 		double x_directional_friction;
 		double y_directional_friction;
 		double angular_directional_friction;

		double battery_voltage;
	};
 */


public class DriveParameters {
	public static final int SIZE = 19;

	public final double eMotorConstant;
	public final double tMotorConstant;
	public final double armatureResistance;
	public final double robotMass;
	public final double robotMoment;
	public final double wheelMoment;
	public final double rollerMoment;
	public final double flWheelFriction;
	public final double frWheelFriction;
	public final double blWheelFriction;
	public final double brWheelFriction;
	public final double flRollerFriction;
	public final double frRollerFriction;
	public final double blRollerFriction;
	public final double brRollerFriction;
	public final double xDirectionalFriction;
	public final double yDirectionalFriction;
	public final double angularDirectionalFriction;
	public final double batteryVoltage;

	public DriveParameters(
		double eMotorConstant, double tMotorConstant, double armatureResistance, double robotMass, double robotMoment,
		double wheelMoment, double rollerMoment, double flWheelFriction, double frWheelFriction,
		double blWheelFriction, double brWheelFriction, double flRollerFriction,
		double frRollerFriction, double blRollerFriction, double brRollerFriction,
		double xDirectionalFriction, double yDirectionalFriction, double angularDirectionalFriction,
		double batteryVoltage
	) {
		this.eMotorConstant = eMotorConstant;
		this.tMotorConstant = tMotorConstant;
		this.armatureResistance = armatureResistance;
		this.robotMass = robotMass;
		this.robotMoment = robotMoment;
		this.wheelMoment = wheelMoment;
		this.rollerMoment = rollerMoment;
		this.flWheelFriction = flWheelFriction;
		this.frWheelFriction = frWheelFriction;
		this.blWheelFriction = blWheelFriction;
		this.brWheelFriction = brWheelFriction;
		this.flRollerFriction = flRollerFriction;
		this.frRollerFriction = frRollerFriction;
		this.blRollerFriction = blRollerFriction;
		this.brRollerFriction = brRollerFriction;
		this.xDirectionalFriction = xDirectionalFriction;
		this.yDirectionalFriction = yDirectionalFriction;
		this.angularDirectionalFriction = angularDirectionalFriction;
        this.batteryVoltage = batteryVoltage;
    }

    public static DriveParameters ofBatteryVoltage(double battery_voltage, DriveParameters previous) {
        return new DriveParameters(
            previous.eMotorConstant,
            previous.tMotorConstant,
            previous.armatureResistance,
            previous.robotMass,
            previous.robotMoment,
            previous.wheelMoment,
            previous.rollerMoment,
            previous.flWheelFriction,
            previous.frWheelFriction,
            previous.blWheelFriction,
            previous.brWheelFriction,
            previous.flRollerFriction,
            previous.frRollerFriction,
            previous.blRollerFriction,
            previous.brRollerFriction,
			previous.xDirectionalFriction,
			previous.yDirectionalFriction,
			previous.angularDirectionalFriction,
            battery_voltage
        );
    }

    public static DriveParameters ofDefaultDriveParameters(double batteryVoltage) {
        return new DriveParameters(
			0.38869198809920497, 0.17413942258926335, 0.9,
			12.999999983125175, 0.01897086956915779,
			0.006733058747799317, 0.00010000000017017123,
			0, 0, 0, 0,
			0, 0, 0, 0,
			10.73229685481671, 36.707741175403896, 7.347775273225521,
			batteryVoltage);
    }

    @SuppressWarnings("DefaultLocale")
	public String toString() {
        return String.format(
			"DriveParameters(motor_constant_e=%.2f, motor_constant_t=%.2f, armature_resistance=%.2f, robot_mass=%.2f, robot_moment=%.2f, wheel_moment=%.2f, roller_moment=%.3f, fl_wheel_friction=%.3f, fr_wheel_friction=%.3f, bl_wheel_friction=%.3f, br_wheel_friction=%.3f, fl_roller_friction=%.2f, fr_roller_friction=%.2f, bl_roller_friction=%.2f, br_roller_friction=%.2f, x_directional_friction=%.2f, y_directional_friction=%.2f, angular_directional_friction=%.2f, batteryVoltage=%.2f)",
			eMotorConstant, tMotorConstant, armatureResistance, robotMass, robotMoment, wheelMoment, rollerMoment,
			flWheelFriction, frWheelFriction, blWheelFriction, brWheelFriction,
			flRollerFriction, frRollerFriction, blRollerFriction, brRollerFriction,
			xDirectionalFriction, yDirectionalFriction, angularDirectionalFriction, batteryVoltage);
    }

    public double[] toArray() {
        return new double[] {
			eMotorConstant, tMotorConstant, armatureResistance, robotMass, robotMoment, wheelMoment, rollerMoment,
			flWheelFriction, frWheelFriction, blWheelFriction, brWheelFriction,
			flRollerFriction, frRollerFriction, blRollerFriction, brRollerFriction,
			xDirectionalFriction, yDirectionalFriction, angularDirectionalFriction,
			batteryVoltage
		};
    }
}