package com.team254.frc2024.commands;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotContainer;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Visualizes a note shot in simulation, including physics and note speed.
 */
public class VisualizeNoteShot extends Command {
    public static Command visualizeNodeShot(RobotContainer container, boolean useGravity) {
        Supplier<MotionParams> supplier = () -> {
            MotionParams params = new MotionParams();
            Pose2d robotPose = container.getRobotState().getLatestFieldToRobot().getValue();

            Rotation2d turretRotation = container.getRobotState().getLatestRobotToTurret().getValue();
            // Redefine turret to be in field frame
            turretRotation = robotPose.getRotation().rotateBy(turretRotation);

            double hoodAngle = Math.toRadians(Constants.HoodConstants.kHoodZeroedAngleDegrees)
                    - container.getHood().getCurrentPosition();
            double shooterMPS = container.getShooterStage2().getCurrentVelocity() *
                    Constants.ShooterConstants.kRingLaunchVelMetersPerSecPerRotPerSec;

            Translation3d velocity = new Translation3d(
                    shooterMPS * Math.cos(hoodAngle), 0.0,
                    shooterMPS * Math.sin(hoodAngle)).rotateBy(
                            new Rotation3d(0.0, 0.0, turretRotation.getRadians()));
            var chassisSpeeds = container.getRobotState().getLatestMeasuredFieldRelativeChassisSpeeds();
            velocity = velocity.plus(new Translation3d(chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond, 0.0));
            params.startPose = new Pose3d(robotPose.getX(),
                    robotPose.getY(),
                    Constants.kNoteReleaseHeight, new Rotation3d(0.0, -hoodAngle, turretRotation.getRadians()));
            params.velocity = velocity;
            return params;
        };
        return new ScheduleCommand(new VisualizeNoteShot(supplier, useGravity));
    }

    public static class MotionParams {
        public Pose3d startPose;
        public Translation3d velocity;
    }

    protected Pose3d pose;
    protected Supplier<MotionParams> paramsSupplier;
    protected MotionParams params;
    protected double lastTimestamp;
    protected boolean useGravity;

    public VisualizeNoteShot(Supplier<MotionParams> paramsSupplier) {
        this(paramsSupplier, false);
    }

    public VisualizeNoteShot(Supplier<MotionParams> paramsSupplier, boolean useGravity) {
        this.paramsSupplier = paramsSupplier;
        this.useGravity = useGravity;
    }

    @Override
    public void initialize() {
        params = paramsSupplier.get();
        pose = params.startPose;
        lastTimestamp = RobotTime.getTimestampSeconds();
    }

    @Override
    public void execute() {
        double timestamp = RobotTime.getTimestampSeconds();
        double dt = timestamp - lastTimestamp;
        pose = new Pose3d(
                pose.getX() + params.velocity.getX() * dt,
                pose.getY() + params.velocity.getY() * dt,
                pose.getZ() + params.velocity.getZ() * dt, pose.getRotation());
        if (useGravity) {
            params.velocity = params.velocity.plus(
                    new Translation3d(0.0, 0.0, -9.8 * dt));
        }
        Logger.recordOutput(
                "NoteVisualizer",
                new Pose3d[] {
                        pose
                });
        lastTimestamp = timestamp;
    }

    @Override
    public boolean isFinished() {
        // We are done if hit ground or bounds of arena
        if (pose.getZ() < 0.0 || pose.getX() < 0.0 || pose.getX() > 20.0 ||
                pose.getZ() > 5.0 || pose.getY() < 0.0 || pose.getY() > 10.0) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
    }
}
