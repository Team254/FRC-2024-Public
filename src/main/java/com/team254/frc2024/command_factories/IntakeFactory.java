package com.team254.frc2024.command_factories;

import java.util.function.DoubleSupplier;

import com.team254.frc2024.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Factory class for creating commands related to the intake subsystem.
 *
 * <p>This class provides a method to create a command for running the intake at a specified speed.
 * The speed is provided through a {@link DoubleSupplier}, allowing for dynamic adjustment of the intake's duty cycle output.
 */

public class IntakeFactory {
    public static Command runIntake(RobotContainer container, DoubleSupplier intakeSpeed) {
        return container.getIntake().dutyCycleCommand(intakeSpeed).withName("Duty Cycle Intake");
    }
}
