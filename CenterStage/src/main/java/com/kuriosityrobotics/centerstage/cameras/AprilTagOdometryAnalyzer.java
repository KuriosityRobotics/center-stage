package com.kuriosityrobotics.centerstage.cameras;

import com.kuriosityrobotics.centerstage.math.Pose;
import com.kuriosityrobotics.centerstage.math.Twist;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Map;
import java.util.Optional;

public enum AprilTagOdometryAnalyzer {
	RED_TEAM(Map.of(
		6, Pose.of(3.42, 0.74, Math.PI),
		5, Pose.of(3.42, 0.89, Math.PI),
		4, Pose.of(3.42, 1.045, Math.PI)
	)),
	BLUE_TEAM(Map.of(
		3, Pose.of(3.42, 1.045, Math.PI).mirror(),
		2, Pose.of(3.42, 0.89, Math.PI).mirror(),
		1, Pose.of(3.42, 0.74, Math.PI).mirror()
	));

	private static final Twist OFFSET = Twist.of(-0.2,0, Math.PI); // offset between camera and robot centre

	private final Map<Integer, Pose> mapping;
	AprilTagOdometryAnalyzer(Map<Integer, Pose> mapping) {
		this.mapping = mapping;
	}

	public Optional<Pose> poseFor(AprilTagDetection detection) {
		Pose pose = mapping.get(detection.id);
		if (pose == null) return Optional.empty();

		var detectionPose = detection.ftcPose;

		if (detectionPose.range < 0.05) return Optional.empty();
//		if (detectionPose.range > 0.5) return Optional.empty();

		double offsetAngle = pose.orientation() - detectionPose.yaw;
		var offset = Twist.of(detectionPose.y, -detectionPose.x, offsetAngle).subtract(OFFSET);

		if (Math.abs(detectionPose.bearing) > Math.toRadians(30)) return Optional.empty();

		return Optional.of(
			pose.add(offset.rotate(offsetAngle))
		);
	}
}
