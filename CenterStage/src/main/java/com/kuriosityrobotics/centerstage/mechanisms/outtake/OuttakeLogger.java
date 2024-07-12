package com.kuriosityrobotics.centerstage.mechanisms.outtake;

import com.kuriosityrobotics.centerstage.bulkdata.BulkDataFetcher;
import com.kuriosityrobotics.centerstage.bulkdata.RevHubBulkData;
import com.kuriosityrobotics.centerstage.util.Instant;

import java.io.FileNotFoundException;
import java.io.PrintWriter;

public class OuttakeLogger {
	private PrintWriter out;

	private final Instant startTime = Instant.now();

	public OuttakeLogger(BulkDataFetcher bulkDataFetcher) {
		try {
			out = new PrintWriter("/sdcard/FIRST/outtake.csv");
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		out.println("time,left_pos,right_pos,left_velocity,right_velocity");

		bulkDataFetcher.addExpansionHubListener(this::updateCSV);
	}

	public synchronized void updateCSV(RevHubBulkData data) {
		out.printf(
			"%f,%d,%d,%d,%d%n",
			Instant.now().since(startTime).toSeconds(),
			-data.encoders[0],
			data.encoders[1],
			-data.velocities[0],
			data.velocities[1]
		);
		out.flush();
	}
}
