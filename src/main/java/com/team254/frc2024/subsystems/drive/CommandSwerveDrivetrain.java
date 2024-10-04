package com.team254.frc2024.subsystems.drive;

import com.team254.lib.ctre.swerve.SwerveDrivetrainConstants;
import com.team254.lib.ctre.swerve.SwerveModuleConstants;

/**
 * This is a simple container for holding CTRE drive creation constants. It is
 * called the
 * same thing that the generated TunerConstants file from TunerX is called.
 * Instead of implementing
 * the drive base here, we simply hold the constants, then use them to create a
 * DriveSubsytem later in
 * RobotContainer. This is done so that we can copy and paste in new
 * TunerConstants.java if we need
 * to change configuration and want to use the wizard again.
 */
public class CommandSwerveDrivetrain {

    SwerveDrivetrainConstants driveTrainConstants;
    SwerveModuleConstants[] moduleConstants;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this.driveTrainConstants = driveTrainConstants;
        this.moduleConstants = modules;
    }

    public SwerveDrivetrainConstants getDriveTrainConstants() {
        return driveTrainConstants;
    }

    public SwerveModuleConstants[] getModuleConstants() {
        return moduleConstants;
    }
}
