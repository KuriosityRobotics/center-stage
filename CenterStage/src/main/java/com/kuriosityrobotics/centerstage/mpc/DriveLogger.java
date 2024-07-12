package com.kuriosityrobotics.centerstage.mpc;

import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.localisation.OdometryIntegrator;
import com.kuriosityrobotics.centerstage.mechanisms.DrivetrainNode;
import com.kuriosityrobotics.centerstage.util.Instant;

import java.io.*;
import java.util.concurrent.Executor;

public class DriveLogger {
	private PrintWriter out;

	private final OdometryIntegrator localisation;
	private final DrivetrainNode drivetrain;
	private final StableVoltageSensor voltageSensor;

	private final BulkDataFetcher.BulkDataNotifier bdn;

	private final Instant startTime = Instant.now();


	public DriveLogger(Executor ses, OdometryIntegrator localisation, DrivetrainNode drivetrain, StableVoltageSensor batterySensor, BulkDataFetcher.BulkDataNotifier bdn) {
		this.localisation = localisation;
		this.drivetrain = drivetrain;
		this.voltageSensor = batterySensor;
		this.bdn = bdn;

		try {
			out = new PrintWriter(new BufferedWriter(new OutputStreamWriter(new FileOutputStream("/sdcard/FIRST/drive_log.csv"))),
				true);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		out.println("time,battery_voltage,fl,fr,bl,br,x_position,y_position,angle,x_velocity,y_velocity,angular_velocity");

		ses.execute(this::updateCSV);
		// adb pull /sdcard/FIRST/drive_log.csv mpc/src/main/python/drive_samples/r2/programmed_drive_x.csv
	}

	public void updateCSV() {
		try {
			while (!Thread.interrupted()) {
				bdn.await();

				var d = localisation.getLocalisation();
				var motorPowers = drivetrain.getMotorPowers();

				out.printf(
					"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f%n",
					Instant.now().since(startTime).toSeconds(),
					voltageSensor.getVoltage(),
					motorPowers.powerFrontLeft(),
					motorPowers.powerFrontRight(),
					motorPowers.powerBackLeft(),
					motorPowers.powerBackRight(),
					d.pose().x(),
					d.pose().y(),
					d.pose().orientation(),
					d.twist().x(),
					d.twist().y(),
					d.twist().angular()
				);
			}
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}

}
