package com.kuriosityrobotics.centerstage.cameras;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class CameraNode {
//	static {
//		System.loadLibrary("apriltag");
//	}
	public CameraNode(ScheduledExecutorService ses, WebcamName camera, VisionProcessor ...processors) {
		VisionPortal.Builder builder = new VisionPortal.Builder();
		builder.setCamera(camera);

		for(VisionProcessor vp : processors){
			builder.addProcessor(vp);
			System.out.println("Adding Processor of" + vp.getClass());
		}

		var result = builder.build();

		ses.submit(() -> {
			try {
				while (!Thread.interrupted()) {
					TimeUnit.DAYS.sleep(999);
				}
			} catch (InterruptedException e) {
				result.close();
			}
		});
	}
}
