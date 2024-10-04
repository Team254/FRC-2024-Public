package com.team254.frc2024.commands.autocommands;

import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.command_factories.AimFactory;
import com.team254.frc2024.command_factories.SuperstructureFactory;
import com.team254.lib.util.ShooterSetpoint;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.Supplier;

/**
 * Auto mode that shoots preload from speaker starting location and runs a path
 * to notes A and B, scoring notes A and B.
 */
public class SpeakerSixNoteAB extends PathFollowingAutoModeCommandGroup {
    public SpeakerSixNoteAB(RobotContainer container) {
        super(container, "Speaker3Plus2_AB", true);
        Supplier<ShooterSetpoint> speakerGoalSupplier = ShooterSetpoint
                .speakerSetpointSupplier(container.getRobotState());
        addCommands(driveCommands.get(0));
        addCommands(driveCommands.get(1));
        addCommands(new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier)).andThen(
                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(container, speakerGoalSupplier)));
    }
}
