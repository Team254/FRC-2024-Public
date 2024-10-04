package com.team254.frc2024.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionFieldPoseEstimate {

    private final Pose2d visionRobotPoseMeters;
    private final double timestampSeconds;
    private final Matrix<N3, N1> visionMeasurementStdDevs;

    public VisionFieldPoseEstimate(Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        this.visionRobotPoseMeters = visionRobotPoseMeters;
        this.timestampSeconds = timestampSeconds;
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }

    public Pose2d getVisionRobotPoseMeters() {
        return visionRobotPoseMeters;
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        return visionMeasurementStdDevs;
    }
}
