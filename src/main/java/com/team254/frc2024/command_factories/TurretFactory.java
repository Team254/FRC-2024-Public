package com.team254.frc2024.command_factories;

import java.util.function.Supplier;

import com.team254.frc2024.RobotContainer;
import com.team254.lib.util.ShooterSetpoint;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Factory class for creating turret control commands for the robot.
 *
 * <p>
 * This class provides static methods to generate commands that control the
 * turret's position and behavior, allowing the turret to aim or move based on
 * various conditions and setpoints.
 */

public class TurretFactory {
    public static Command aimTurretToPose(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        var turret = container.getTurret();
        return turret.positionSetpointCommand(() -> setpointSupplier.get().getTurretRadiansFromCenter(),
                () -> setpointSupplier.get().getTurretFF()).withName("Align Turret To Pose");
    }
}
