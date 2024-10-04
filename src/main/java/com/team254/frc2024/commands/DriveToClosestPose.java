package com.team254.frc2024.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.team254.frc2024.RobotState;
import com.team254.frc2024.subsystems.drive.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Drivest to the closest pose in a list of poses.
 */
public class DriveToClosestPose extends DriveToPose {

	public DriveToClosestPose(DriveSubsystem driveSubsystem, RobotState robotState,
			Supplier<List<Pose2d>> poseSupplierList) {
		super(driveSubsystem, robotState, () -> {
			Optional<Pose2d> closestPose = Optional.empty();
			double closestDistance = 20.0; // m
			for (var targetPose : poseSupplierList.get()) {
				double currentDistance = targetPose.relativeTo(robotState.getLatestFieldToRobot().getValue())
						.getTranslation().getNorm();
				if (currentDistance < closestDistance) {
					closestDistance = currentDistance;
					closestPose = Optional.of(targetPose);
				}
			}
			if (closestPose.isEmpty())
				return null;
			return closestPose.get();
		});
	}

}
