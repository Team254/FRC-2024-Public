package com.team254.frc2024.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.team254.lib.ctre.swerve.SwerveModule;
import com.team254.lib.ctre.swerve.SwerveRequest;
import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.Constants;
import com.team254.frc2024.Robot;
import com.team254.frc2024.RobotState;
import com.team254.frc2024.subsystems.drive.DriveSubsystem;
import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Heading controller drive command.
 */
public class DriveMaintainingHeadingCommand extends Command {
    public DriveMaintainingHeadingCommand(DriveSubsystem drivetrain, RobotState robotState, DoubleSupplier throttle,
            DoubleSupplier strafe, DoubleSupplier turn) {
        mDrivetrain = drivetrain;
        mRobotState = robotState;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;

        driveWithHeading.HeadingController.setPID(Constants.DriveConstants.kHeadingControllerP,
                Constants.DriveConstants.kHeadingControllerI, Constants.DriveConstants.kHeadingControllerD);
        driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // uncomment for easy PID tuning
        // SmartDashboard.putData("Heading Controller PID",
        // driveWithHeading.HeadingController);

        addRequirements(drivetrain);
        setName("Swerve Drive Maintain Heading");

        if (Robot.isSimulation()) {
            driveNoHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
            driveWithHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        }
    }

    private RobotState mRobotState;
    private DriveSubsystem mDrivetrain;
    private DoubleSupplier mThrottleSupplier, mStrafeSupplier, mTurnSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;

    private SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DriveConstants.kDriveMaxSpeed * 0.05) // Add a 5% deadband in open loop
            .withRotationalDeadband(Constants.DriveConstants.kDriveMaxAngularRate * Constants.kSteerJoystickDeadband)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.DriveConstants.kDriveMaxSpeed * 0.05)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * Constants.DriveConstants.kDriveMaxSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * Constants.DriveConstants.kDriveMaxSpeed;
        double turnFieldFrame = mTurnSupplier.getAsDouble();
        double throttleFieldFrame = mRobotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldFrame = mRobotState.isRedAlliance() ? -strafe : strafe;
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband
                || (Util.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
                        && Math.abs(mRobotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond) > Math
                                .toRadians(10))) {
            turnFieldFrame = turnFieldFrame * Constants.DriveConstants.kDriveMaxAngularRate;
            mDrivetrain.setControl(driveNoHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                    .withRotationalRate(turnFieldFrame));
            mHeadingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (mHeadingSetpoint.isEmpty()) {
                mHeadingSetpoint = Optional.of(mRobotState.getLatestFieldToRobot().getValue().getRotation());
            }
            mDrivetrain.setControl(driveWithHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                    .withTargetDirection(mHeadingSetpoint.get()));
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput("DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}