package com.team254.frc2024.command_factories;

import java.util.function.Supplier;

import com.team254.frc2024.RobotContainer;
import com.team254.lib.util.ShooterSetpoint;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Factory class for creating commands related to the hood subsystem.
 *
 * <p>
 * This class provides static methods to create commands for controlling the
 * robot's hood.
 * It includes a method to aim the hood to a specific position based on the
 * provided shooter setpoints.
 */

public class HoodFactory {
    public static Command aimHoodToPose(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        var hood = container.getHood();
        return hood.positionSetpointCommand(() -> setpointSupplier.get().getHoodRadians(),
                () -> setpointSupplier.get().getHoodFF())
                .withName("Align Hood To Pose");
    }
}
