package com.team254.frc2024.command_factories;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotContainer;
import com.team254.lib.util.ShooterSetpoint;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Factory class for creating commands related to aiming and aligning the
 * robot's superstructure.
 *
 * <p>
 * Provides methods to create commands that handle the alignment of the turret,
 * hood, and shooter
 * for precise targeting. This includes commands for aligning individual
 * components as well as a combined
 * command to align the entire superstructure until it's on target.
 */

public class AimFactory {
    public static Command alignSuperstructure(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        return new ParallelCommandGroup(
                alignHoodAndTurret(container, setpointSupplier),
                ShooterFactory.spinUpStage2(container,
                        setpointSupplier))
                .withName("Align Superstructure");
    }

    public static Command alignHoodAndTurret(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        return new ParallelCommandGroup(
                TurretFactory.aimTurretToPose(container, setpointSupplier),
                HoodFactory.aimHoodToPose(container, setpointSupplier))
                .withName("Align Hood and Turret");
    }

    public static Command alignUntilOnTarget(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        return alignSuperstructure(container, setpointSupplier)
                .until(() -> onTarget(container,
                        setpointSupplier))
                .withName("Align Until On Target");
    }

    public static boolean onTarget(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        var turret = container.getTurret();
        var hood = container.getHood();
        var shooterStage2 = container.getShooterStage2();
        double shooterSetpoint = setpointSupplier.get().getShooterRPS();
        boolean shooterOnTarget = Util.epsilonEquals(shooterStage2.getCurrentVelocity(), shooterSetpoint,
                shooterSetpoint * 0.04);
        boolean turretOnTarget = Math.abs(new Rotation2d(turret.getCurrentPosition()).rotateBy(
                new Rotation2d(setpointSupplier.get().getTurretRadiansFromCenter()).unaryMinus())
                .getRadians()) < Constants.TurretConstants.kTurretShootingEpsilon;
        boolean hoodOnTarget = Util.epsilonEquals(hood.getCurrentPosition(), setpointSupplier.get().getHoodRadians(),
                Constants.HoodConstants.kHoodShootingEpsilon);
        boolean isValid = setpointSupplier.get().getIsValid();
        Logger.recordOutput("Shooter/IsValid", isValid);
        Logger.recordOutput("Shooter/OnTarget", shooterOnTarget);
        Logger.recordOutput("Turret/OnTarget", turretOnTarget);
        Logger.recordOutput("Hood/OnTarget", hoodOnTarget);
        return hoodOnTarget && shooterOnTarget && turretOnTarget && isValid;
    }

}
