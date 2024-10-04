package com.team254.frc2024.commands.autocommands;

import com.team254.frc2024.RobotContainer;

/**
 * Test auto mode.
 */
public class TestAuto extends PathFollowingAutoModeCommandGroup {
    public TestAuto(RobotContainer container) {
        super(container, "TestAuto", false);
        addCommands(driveCommands.get(0));
    }
}
