package com.team254.frc2024.commands.autocommands;

import com.team254.frc2024.RobotContainer;

/**
 * Auto mode that shoots preload and backs off to midline.
 */
public class PreloadBackoff extends PathFollowingAutoModeCommandGroup {
    public PreloadBackoff(RobotContainer container) {
        super(container, "PreloadBackoff", false);
        addCommands(driveCommands.get(0));
    }
}
