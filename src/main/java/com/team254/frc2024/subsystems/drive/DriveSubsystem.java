package com.team254.frc2024.subsystems.drive;

import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team254.frc2024.Constants;
import com.team254.frc2024.Robot;
import com.team254.frc2024.RobotState;
import com.team254.frc2024.subsystems.vision.VisionFieldPoseEstimate;

import com.team254.lib.ctre.swerve.SwerveRequest;
import com.team254.lib.pathplanner.AdvancedAutoBuilder;
import com.team254.lib.pathplanner.AdvancedHolonomicPathFollowerConfig;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team254.lib.ctre.swerve.SwerveModule.DriveRequestType;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
    DriveIO io;

    DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    DriveViz telemetry = new DriveViz(Constants.DriveConstants.kDriveMaxSpeed);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    RobotState robotState;

    public DriveSubsystem(DriveIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        if (Robot.isSimulation()) {
            autoRequest.DriveRequestType = DriveRequestType.OpenLoopVoltage;
        }

        configurePathPlanner();
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.readInputs(inputs);
        telemetry.telemeterize(inputs);
        Logger.processInputs("DriveInputs", inputs);
        io.logModules(inputs);
        robotState.incrementIterationCount();
        Logger.recordOutput("Drive/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    public void resetOdometry(Pose2d pose) {
        io.resetOdometry(pose);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AdvancedAutoBuilder.configureHolonomic(
                robotState::getLatestFieldToRobotCenter,
                (pose) -> {
                },
                // this::seedFieldRelative,
                () -> robotState.getLatestRobotRelativeChassisSpeed(),
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new AdvancedHolonomicPathFollowerConfig(new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
                        Constants.AutoConstants.kTranslationKa,
                        CompTunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().equals(Optional.of(Alliance.Red)),
                this);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/targetPose", pose);
        });

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/currentPose", pose);
        });
    }

    public Pose2d getEstimatedPosition() {
        return io.getEstimatedPosition();
    }

    public Translation2d[] getModuleLocations() {
        return io.getModuleLocations();
    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    // API
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
    }

    public void seedFieldRelative() {
        io.seedFieldRelative();
    }

    public void seedFieldRelative(Pose2d location) {
        io.seedFieldRelative(location);
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        if (Robot.isReal()) {
            io.addVisionMeasurement(visionFieldPoseEstimate);
        }
    }

}
