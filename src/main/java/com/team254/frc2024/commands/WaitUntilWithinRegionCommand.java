package com.team254.frc2024.commands;

import com.team254.frc2024.RobotState;
import com.team254.lib.auto.AutoUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Waits until the robot is within a region defined by two points.
 */
public class WaitUntilWithinRegionCommand extends Command {

    private Translation2d bottomLeft, topRight;
    private RobotState robotState;
    private boolean useAlliance;

    private boolean isInRegion = false;

    public WaitUntilWithinRegionCommand(RobotState robotState, Translation2d bottomLeft,
            Translation2d topRight, boolean useAlliance) {
        this.bottomLeft = bottomLeft;
        this.topRight = topRight;
        this.robotState = robotState;
        this.useAlliance = useAlliance;
        setName("Wait Until Within Region");
    }

    @Override
    public void initialize() {
        isInRegion = false;
    }

    @Override
    public void execute() {
        Pose2d drivePose = robotState.getLatestFieldToRobot().getValue();
        if (useAlliance && robotState.isRedAlliance()) {
            drivePose = new Pose2d(AutoUtil.flipTranslation(drivePose.getTranslation()), drivePose.getRotation());
        }
        isInRegion = drivePose.getTranslation().getX() < topRight.getX()
                && drivePose.getTranslation().getX() > bottomLeft.getX()
                && drivePose.getTranslation().getY() < bottomLeft.getY()
                && drivePose.getTranslation().getY() > topRight.getY();
    }

    @Override
    public boolean isFinished() {
        return isInRegion;
    }
}
