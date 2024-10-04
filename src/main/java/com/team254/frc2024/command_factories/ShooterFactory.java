package com.team254.frc2024.command_factories;

import java.util.function.Supplier;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotContainer;
import com.team254.lib.util.ShooterSetpoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Factory class for creating commands related to the shooter subsystem.
 *
 * <p>
 * This class provides various methods to create commands for controlling the
 * top and bottom stages of the shooter.
 * These methods include commands to spin up different stages of the shooter,
 * intake until a note is staged, and exhaust(release) a note from stage 1.
 */

public class ShooterFactory {
    public static Command spinUpStage2(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        var shooterStage2 = container.getShooterStage2();
        return shooterStage2.velocitySetpointCommand(
                () -> setpointSupplier.get().getShooterRPS()).withName("Spin Up Stage 2 Shooter");
    }

    public static Command spinBothStages(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        var shooterStage1 = container.getShooterStage1();
        var shooterStage2 = container.getShooterStage2();
        return new ParallelCommandGroup(shooterStage1.velocitySetpointCommand(
                () -> setpointSupplier.get().getShooterStage1RPS()),
                shooterStage2.velocitySetpointCommand(
                        () -> setpointSupplier.get().getShooterRPS()))
                .withName("Spin Both Shooter Stages");
    }

    public static Command spinOnlyStageOne(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        var shooterStage1 = container.getShooterStage1();
        return shooterStage1.velocitySetpointCommand(
                () -> setpointSupplier.get().getShooterStage1RPS()).withName("Spin Up Stage 1 Shooter");
    }

    /**
     * Runs stage 1 forward to stow a note in it.
     * Exits: When the note is stowed, according to banner sensor
     */
    public static Command intakeUntilStagedInStage1(RobotContainer container) {
        var shooterStage1 = container.getShooterStage1();
        return shooterStage1.runUntilBanner(() -> Constants.ShooterConstants.kShooterStage1IntakeRPS).andThen(
                shooterStage1.velocitySetpointCommand(() -> 0.0).withTimeout(0.01));
    }

    /**
     * Exhausts a note from stage 1.
     * Exits: When interrupted
     */
    public static Command exhaustStage1(RobotContainer container) {
        var shooterStage1 = container.getShooterStage1();
        return shooterStage1.velocitySetpointCommand(() -> Constants.ShooterConstants.kShooterStage1ExhaustRPS);
    }
}
