package com.team254.frc2024.commands.autocommands;

import java.util.function.Supplier;

import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.command_factories.AimFactory;
import com.team254.frc2024.command_factories.SuperstructureFactory;
import com.team254.lib.util.ShooterSetpoint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Truss auto mode that shoots preload, three close, and notes B and A from
 * midline.
 */
public class TrussSixNoteBA extends PathFollowingAutoModeCommandGroup {
    public TrussSixNoteBA(RobotContainer container) {
        super(container, "Truss3Plus2_BA", true);
        Supplier<ShooterSetpoint> speakerGoalSupplier = ShooterSetpoint
                .speakerSetpointSupplier(container.getRobotState());

        addCommands(new ParallelCommandGroup(
                new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier)).andThen(
                        SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(container, speakerGoalSupplier)
                                .until(() -> !container.getShooterStage1().hasNote())),
                driveCommands.get(0)));
        addCommands(driveCommands.get(1));
        addCommands(driveCommands.get(2));
        addCommands(new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier)).andThen(
                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(container, speakerGoalSupplier)));
    }
}
