package com.team254.frc2024.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team254.lib.ctre.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.team254.lib.util.MathHelpers;
import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotState;
import com.team254.frc2024.subsystems.drive.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives to a specified pose.
 */
public class DriveToPose extends Command {
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            Constants.AutoConstants.kPXController, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private DriveSubsystem driveSubsystem;
    private Supplier<Pose2d> poseSupplier;
    private RobotState robotState;
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

    public DriveToPose(DriveSubsystem driveSubsystem, RobotState robotState, Supplier<Pose2d> poseSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.robotState = robotState;
        this.poseSupplier = poseSupplier;
        addRequirements(driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
        driveController.reset(
                currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(robotState.getLatestMeasuredFieldRelativeChassisSpeeds().vxMetersPerSecond,
                                robotState.getLatestMeasuredFieldRelativeChassisSpeeds().vyMetersPerSecond)
                                .rotateBy(
                                        poseSupplier
                                                .get()
                                                .getTranslation()
                                                .minus(robotState.getLatestFieldToRobot().getValue().getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(),
                robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        lastSetpointTranslation = robotState.getLatestFieldToRobot().getValue().getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentPose = robotState.getLatestFieldToRobot().getValue();
        Pose2d targetPose = poseSupplier.get();

        Logger.recordOutput("DriveToPose/currentPose", currentPose);
        Logger.recordOutput("DriveToPose/targetPose", targetPose);

        double currentDistance = currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        MathHelpers.transform2dFromTranslation(
                                new Translation2d(driveController.getSetpoint().position, 0.0)))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity = MathHelpers
                .pose2dFromRotation(currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(MathHelpers.transform2dFromTranslation(new Translation2d(driveVelocityScalar, 0.0)))
                .getTranslation();
        driveSubsystem.setControl(new ApplyChassisSpeeds().withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation())));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setControl(new ApplyChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return poseSupplier.get().equals(null) || (driveController.atGoal() && thetaController.atGoal());
    }
}
