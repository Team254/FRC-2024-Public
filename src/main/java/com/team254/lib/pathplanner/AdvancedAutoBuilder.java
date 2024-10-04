package com.team254.lib.pathplanner;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AdvancedAutoBuilder {
    /** An exception while building autos */
    public static class AutoBuilderException extends RuntimeException {
        /**
         * Create a new auto builder exception
         *
         * @param message Error message
         */
        public AutoBuilderException(String message) {
            super(message);
        }
    }

    private static boolean configured = false;

    private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;

    /**
     * Configures the AutoBuilder for a holonomic drivetrain.
     *
     * @param poseSupplier                a supplier for the robot's current pose
     * @param resetPose                   a consumer for resetting the robot's pose
     * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot
     *                                    relative chassis
     *                                    speeds
     * @param robotRelativeOutput         a consumer for setting the robot's
     *                                    robot-relative chassis speeds
     * @param config                      {@link com.pathplanner.lib.util.HolonomicPathFollowerConfig}
     *                                    for configuring the
     *                                    path following commands
     * @param shouldFlipPath              Supplier that determines if paths should
     *                                    be flipped to the other side of
     *                                    the field. This will maintain a global
     *                                    blue alliance origin.
     * @param driveSubsystem              the subsystem for the robot's drive
     */
    public static void configureHolonomic(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            Consumer<ChassisSpeeds> robotRelativeOutput,
            AdvancedHolonomicPathFollowerConfig config,
            BooleanSupplier shouldFlipPath,
            Subsystem driveSubsystem) {
        if (configured) {
            DriverStation.reportError(
                    "Auto builder has already been configured. This is likely in error.", true);
        }

        AdvancedAutoBuilder.pathFollowingCommandBuilder = (path) -> new AdvancedFollowPathHolonomic(
                path,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
                shouldFlipPath,
                driveSubsystem);
        AdvancedAutoBuilder.configured = true;
    }

    /**
     * Returns whether the AutoBuilder has been configured.
     *
     * @return true if the AutoBuilder has been configured, false otherwise
     */
    public static boolean isConfigured() {
        return configured;
    }

    /**
     * Builds a command to follow a path. PathPlannerLib commands will also trigger
     * event markers
     * along the way.
     *
     * @param path the path to follow
     * @return a path following command with for the given path
     * @throws AutoBuilderException if the AutoBuilder has not been configured
     */
    public static Command followPath(PathPlannerPath path) {
        if (!isConfigured()) {
            throw new AutoBuilderException(
                    "Auto builder was used to build a path following command before being configured");
        }

        return pathFollowingCommandBuilder.apply(path);
    }

}
