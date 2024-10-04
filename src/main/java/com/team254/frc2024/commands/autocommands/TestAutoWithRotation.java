package com.team254.frc2024.commands.autocommands;

import com.team254.frc2024.RobotContainer;

/**
 * Test auto mode wth rotation.
 */
public class TestAutoWithRotation extends PathFollowingAutoModeCommandGroup {
    public TestAutoWithRotation(RobotContainer container) {
        super(container, "TestAutoWithRotation", false);
        addCommands(driveCommands.get(0));
    }
}
