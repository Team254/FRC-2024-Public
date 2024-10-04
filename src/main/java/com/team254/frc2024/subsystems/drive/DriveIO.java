package com.team254.frc2024.subsystems.drive;

import com.team254.frc2024.subsystems.vision.VisionFieldPoseEstimate;

import com.team254.lib.ctre.swerve.SwerveDrivetrain;
import com.team254.lib.ctre.swerve.SwerveRequest;
import com.team254.lib.util.MathHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

public interface DriveIO {

    @AutoLog
    class DriveIOInputs extends SwerveDrivetrain.SwerveDriveState {
        public double gyroAngle = 0.0;

        DriveIOInputs() {
            this.Pose = MathHelpers.kPose2dZero;
        }

        public void fromSwerveDriveState(SwerveDrivetrain.SwerveDriveState stateIn) {
            this.Pose = stateIn.Pose;
            this.SuccessfulDaqs = stateIn.SuccessfulDaqs;
            this.FailedDaqs = stateIn.FailedDaqs;
            this.ModuleStates = stateIn.ModuleStates;
            this.ModuleTargets = stateIn.ModuleTargets;
            this.speeds = stateIn.speeds;
            this.OdometryPeriod = stateIn.OdometryPeriod;
        }
    }

    void readInputs(DriveIOInputs inputs);

    void logModules(SwerveDrivetrain.SwerveDriveState driveState);

    void seedFieldRelative();

    void seedFieldRelative(Pose2d location);

    void resetOdometry(Pose2d pose);

    Pose2d getEstimatedPosition();

    Translation2d[] getModuleLocations();

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate);
}
