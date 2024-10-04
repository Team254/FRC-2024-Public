package com.team254.frc2024.command_factories;

import com.team254.frc2024.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.function.DoubleSupplier;

/**
 * Factory class for creating commands related to the feeder subsystem.
 *
 * <p>
 * This class provides static methods to create commands for controlling the
 * feeder system of the robot.
 * It includes methods for running both feeders towards or away from the
 * shooter, and for controlling the feeders
 * in a specific direction with custom speeds.
 */

public class FeederFactory {
    public static Command runBothFeedersTowardsShooter(RobotContainer container, DoubleSupplier rps) {
        var feeder = container.getFeeder();
        return feeder.intakeUnjam(container, rps);
    }

    public static Command runBothFeedersAwayFromShooter(RobotContainer container, DoubleSupplier rps) {
        return runBothFeedersTowardsShooter(container, () -> -rps.getAsDouble());
    }

    // double supplier positive + is clockwise -> negative left, positive right
    public static Command runDirection(RobotContainer container, boolean isClockwise, DoubleSupplier rpsLeft,
            DoubleSupplier rpsRight) {
        var feeder = container.getFeeder();
        return new ParallelCommandGroup(
                feeder.velocitySetpointCommand(() -> rpsLeft.getAsDouble() * (isClockwise ? -1 : 1),
                        () -> rpsRight.getAsDouble() * (isClockwise ? 1 : -1)))
                .withName("Feeder " + (isClockwise ? "CW" : "CCW") + " RPS");
    }
}
