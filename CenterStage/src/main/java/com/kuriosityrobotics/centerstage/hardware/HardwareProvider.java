package com.kuriosityrobotics.centerstage.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.*;

/**
 * The HardwareProvider interface provides methods to retrieve different types of hardware devices
 * and controllers for a given LynxHub.
 */
public interface HardwareProvider {
	/**
	 * Returns the LynxModule associated with the given LynxHub.
	 *
	 * @param hub The LynxHub for which to retrieve the LynxModule.
	 * @return The LynxModule associated with the given LynxHub.
	 */
	LynxModule moduleFor(LynxHub hub);

	/**
	 * Retrieves the DcMotorControllerEx for the given LynxHub.
	 *
	 * @param hub The LynxHub for which to retrieve the DcMotorControllerEx.
	 * @return The DcMotorControllerEx associated with the given LynxHub, or null if an exception occurs.
	 */
	DcMotorControllerEx motorControllerFor(LynxHub hub);

	/**
	 * Returns the AnalogInputController for the given LynxHub.
	 *
	 * @param hub The LynxHub for which to retrieve the AnalogInputController.
	 * @return The AnalogInputController associated with the given LynxHub.
	 */
	AnalogInputController analogInputControllerFor(LynxHub hub);

	/**
	 * Retrieves the ServoControllerEx for the given LynxHub.
	 *
	 * @param hub The LynxHub for which to retrieve the ServoControllerEx.
	 * @return The ServoControllerEx associated with the given LynxHub.
	 */
	ServoControllerEx servoControllerFor(LynxHub hub);

	/**
	 * Creates a DcMotorEx object with the specified LynxHub, port number, and direction.
	 *
	 * @param hub        The LynxHub to which the motor is connected.
	 * @param portNumber The port number of the motor.
	 * @param direction  The direction of the motor rotation.
	 * @return A DcMotorEx object representing the motor.
	 */
	DcMotorEx motor(LynxHub hub, int portNumber, DcMotorSimple.Direction direction);

	/**
	 * Creates a ServoImplEx object for the specified LynxHub and port number.
	 *
	 * @param hub        The LynxHub for which to create the ServoImplEx object.
	 * @param portNumber The port number on the LynxHub to which the servo is connected.
	 * @return A ServoImplEx object.
	 */
	ServoImplEx servo(LynxHub hub, int portNumber);

	/**
	 * Creates a CRServo object for the given LynxHub and port number.
	 *
	 * @param hub        The LynxHub for which to create the CRServo object.
	 * @param portNumber The port number to which the CRServo is connected.
	 * @return The CRServo object created for the given LynxHub and port number.
	 */
	CRServoImplEx crServo(LynxHub hub, int portNumber);

	/**
	 * Retrieves the AnalogInput associated with the given LynxHub and channel.
	 *
	 * @param hub     The LynxHub for which to retrieve the AnalogInput.
	 * @param channel The channel of the AnalogInput.
	 * @return The AnalogInput associated with the given LynxHub and channel.
	 */
	AnalogInput analogInput(LynxHub hub, int channel);

	/**
	 * Retrieves an I2cDeviceSynchSimple for the specified LynxHub and port.
	 *
	 * @param hub  The LynxHub to retrieve the I2cDeviceSynchSimple from.
	 * @param port The port number of the I2cDeviceSynchSimple.
	 * @return The requested I2cDeviceSynchSimple.
	 */
	I2cDeviceSynchSimple i2cDevice(LynxHub hub, int port);

	/**
	 * Retrieves a hardware device by its class/interface and device name.
	 *
	 * @param <T>              the type of the hardware device to retrieve
	 * @param classOrInterface the class/interface of the hardware device to retrieve
	 * @param deviceName       the name of the hardware device to retrieve
	 * @return the hardware device of the specified class/interface and name, or null if not found
	 */
	<T extends HardwareDevice> T byName(Class<? extends T> classOrInterface, String deviceName);

	/**
	 * Retrieves the hardware device with the specified device name.
	 *
	 * @param <T>        The type of hardware device to be retrieved.
	 * @param deviceName The name of the device to be retrieved.
	 * @return The hardware device with the specified device name.
	 */
	<T extends HardwareDevice> T byName(String deviceName);

	VoltageSensor voltageSensorFor(String deviceName);
	VoltageSensor voltageSensorFor(LynxHub hub);
}