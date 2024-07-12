package com.kuriosityrobotics.centerstage.hardware;

import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class HardwareProviderImpl implements HardwareProvider {
	private final HardwareMap hardwareMap;

	public HardwareProviderImpl(HardwareMap hardwareMap) {
		this.hardwareMap = hardwareMap;
	}

	private final Map<LynxHub, LynxModule> modules = new ConcurrentHashMap<>();
	private final Map<LynxModule, LynxDcMotorController> motorControllers = new ConcurrentHashMap<>();
	private final Map<LynxModule, AnalogInputController> analogInputControllers = new ConcurrentHashMap<>();
	private final Map<LynxModule, ServoControllerEx> servoControllers = new ConcurrentHashMap<>();

	@Override
	public LynxModule moduleFor(LynxHub hub) {
		return modules.computeIfAbsent(hub, k -> hardwareMap.get(LynxModule.class, k.hardwareName()));
	}

	@Override
	public DcMotorControllerEx motorControllerFor(LynxHub hub) {
		var module = moduleFor(hub);
		return motorControllers.computeIfAbsent(module, m -> {
			try {
				return new LynxDcMotorController(hardwareMap.appContext, m);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				throw new RuntimeException(e);
			} catch (RobotCoreException e) {
				throw new RuntimeException(e);
			}
		});
	}

	@Override
	public AnalogInputController analogInputControllerFor(LynxHub hub) {
		var module = moduleFor(hub);
		return analogInputControllers.computeIfAbsent(module, m -> {
			try {
				return new LynxAnalogInputController(hardwareMap.appContext, m);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				throw new RuntimeException(e);
			} catch (RobotCoreException e) {
				throw new RuntimeException(e);
			}
		});
	}

	@Override
	public ServoControllerEx servoControllerFor(LynxHub hub) {
		var module = moduleFor(hub);
		return servoControllers.computeIfAbsent(module, m -> {
			try {
				return new LynxServoController(hardwareMap.appContext, m);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				throw new RuntimeException(e);
			} catch (RobotCoreException e) {
				throw new RuntimeException(e);
			}
		});
	}

	@Override
	public DcMotorEx motor(LynxHub hub, int portNumber, DcMotorSimple.Direction direction) {
		return new DcMotorImplEx(motorControllerFor(hub), portNumber, direction);
	}

	@Override
	public ServoImplEx servo(LynxHub hub, int portNumber) {
		return new ServoImplEx(servoControllerFor(hub), portNumber, ServoConfigurationType.getStandardServoType());
	}

	@Override
	public CRServoImplEx crServo(LynxHub hub, int portNumber) {
		return new CRServoImplEx(servoControllerFor(hub), portNumber, ServoConfigurationType.getStandardServoType());
	}

	@Override
	public AnalogInput analogInput(LynxHub hub, int channel) {
		return new AnalogInput(analogInputControllerFor(hub), channel);
	}

	@Override
	public I2cDeviceSynchSimple i2cDevice(LynxHub hub, int busNumber) {
		return new LynxI2cDeviceSynchV2(hardwareMap.appContext, moduleFor(hub), busNumber);
	}

	@Override
	public <T extends HardwareDevice> T byName(Class<? extends T> classOrInterface, String deviceName) {
		return hardwareMap.get(classOrInterface, deviceName);
	}

	@Override
	public <T extends HardwareDevice> T byName(String deviceName) {
		return (T) hardwareMap.get(deviceName);
	}

	@Override
	public VoltageSensor voltageSensorFor(String deviceName) {
		return hardwareMap.voltageSensor.get(deviceName);
	}

	@Override
	public VoltageSensor voltageSensorFor(LynxHub hub) {
		try {
			return new LynxVoltageSensor(hardwareMap.appContext, moduleFor(hub));
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			throw new RuntimeException(e);
		} catch (RobotCoreException e) {
			throw new RuntimeException(e);
		}
	}
}