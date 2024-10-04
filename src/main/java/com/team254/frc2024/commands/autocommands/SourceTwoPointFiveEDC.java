package com.team254.frc2024.commands.autocommands;

import java.util.function.Supplier;

import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.command_factories.AimFactory;
import com.team254.frc2024.command_factories.SuperstructureFactory;
import com.team254.lib.util.ShooterSetpoint;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Auto mode that shoots preload and runs a source path to notes E, D, and C,
 * scoring notes E and D.
 */
public class SourceTwoPointFiveEDC extends PathFollowingAutoModeCommandGroup {
    public SourceTwoPointFiveEDC(RobotContainer container) {
        super(container, "Source2-5_EDC", true);
        Supplier<ShooterSetpoint> speakerGoalSupplier = ShooterSetpoint
                .speakerSetpointSupplier(container.getRobotState());
        addCommands(new ParallelCommandGroup(
                new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier)).andThen(
                        SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(container, speakerGoalSupplier)
                                .until(() -> !container.getShooterStage1().hasNote())),
                driveCommands.get(0)));
        addCommands(driveCommands.get(1));
    }
}
