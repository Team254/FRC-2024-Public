// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2024;

import com.team254.frc2024.controlboard.ModalControls;
import com.team254.lib.auto.AutoUtil.FirstAction;
import com.team254.lib.auto.AutoUtil.LastAction;
import com.team254.lib.auto.AutoUtil.PriorityMidlineSequence;
import com.team254.lib.auto.AutoUtil.StartingLocation;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.MathHelpers;
import com.team254.lib.util.OSUtil;
import com.team254.frc2024.commands.autocommands.PathFollowingAutoModeCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.team254.frc2024.AutoModeSelector.DesiredMode;

public class Robot extends LoggedRobot {
    private Command autonomousCommand = Commands.none();
    private Optional<Pose2d> startingPose = Optional.empty();
    private Command disabledCommand = Commands.none();
    private String blockedNotes = "";
    private DesiredMode desiredMode = null;
    private PriorityMidlineSequence priorityMidlineSequence = null;
    private StartingLocation startingLocation = null;
    private FirstAction firstAction = null;
    private LastAction lastAction = null;
    private Optional<Alliance> allianceColor = Optional.of(Alliance.Blue);
    private double lastTimestampNotValid = 0;

    private RobotContainer robotContainer;
    private int mIter = 0;
    private boolean mHasBeenEnabled = false;

    private double timeOfLastSync = 0.0;
    private boolean hasBeenEnabledTeleop = false;
    private boolean hasSetNoOp = false;

    @Override
    public void robotInit() {
        // Init advantage kit
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // always log to usb for now
        Logger.addDataReceiver(new WPILOGWriter());
        if (DriverStation.isFMSAttached()) {
            // Only turn on logs if we are attached to FMS, but disable NTtables.
            // Logger.addDataReceiver(new WPILOGWriter());
        } else {
            // Otherwise, enable NT tables.
            Logger.addDataReceiver(new NT4Publisher());
        }

        if (Constants.kIsReplay) {
            String replayLogPath = LogFileUtil.findReplayLog();

            Logger.setReplaySource(new WPILOGReader(replayLogPath));
        }
        if (Constants.kIsReplay) {
            setUseTiming(true);
        }
        Logger.disableDeterministicTimestamps();
        Logger.start();

        robotContainer = new RobotContainer();

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.getRobotState().updateViz();
        robotContainer.getRobotState().updateLogger();
        if (Robot.isSimulation()) {
            robotContainer.getSimulatedRobotState().updateSim();
        }

        // Do this here instead of teleopPeriodic because of ordering effects.
        if (isTeleopEnabled() && !hasSetNoOp) {
            robotContainer.getModalControls().setMode(ModalControls.Mode.NOT_SPECIFIED);
            hasSetNoOp = true;
        }
    }

    @Override
    public void disabledInit() {
        disabledCommand = robotContainer.getDisabledCommand();
        disabledCommand.schedule();
        timeOfLastSync = Timer.getFPGATimestamp();
    }

    @Override
    public void disabledPeriodic() {
        if (mIter % 50 == 0) {
            if (robotContainer.isValidAutoCommand()) {
                SmartDashboard.putNumber("Time Since Auto Command Not Valid",
                        RobotTime.getTimestampSeconds() - lastTimestampNotValid);
            } else {
                SmartDashboard.putNumber("Time Since Auto Command Not Valid", 0);
                lastTimestampNotValid = RobotTime.getTimestampSeconds();
            }
            if (startingPose.isPresent() && robotContainer.odometryCloseToPose(startingPose.get())) {
                SmartDashboard.putBoolean("Near Auto Starting Pose", true);
            } else {
                SmartDashboard.putBoolean("Near Auto Starting Pose", false);
            }
            DesiredMode latestDesiredMode = robotContainer.getModeChooser().get();
            PriorityMidlineSequence latestPriorityMidlineSequence = robotContainer.getPriorityMidlineSequenceChooser()
                    .get();
            StartingLocation latestStartingLocation = robotContainer.getStartingLocationChooser().get();
            FirstAction latestFirstAction = robotContainer.getFirstActionChooser().get();
            LastAction latestLastAction = robotContainer.getLastActionChooser().get();
            String latestBlockedNotes = robotContainer.getBlockedMidlineNotes().get();
            if (!blockedNotes.equals(latestBlockedNotes) || desiredMode != latestDesiredMode
                    || startingLocation != latestStartingLocation
                    || priorityMidlineSequence != latestPriorityMidlineSequence
                    || firstAction != latestFirstAction || lastAction != latestLastAction
                    || (DriverStation.getAlliance().isPresent()
                            && allianceColor.get() != DriverStation.getAlliance().get())) {
                autonomousCommand = robotContainer.getAutonomousCommand();
                if (autonomousCommand instanceof PathFollowingAutoModeCommandGroup) {
                    startingPose = ((PathFollowingAutoModeCommandGroup) autonomousCommand).getStartingPose();
                    if (startingPose.isPresent()) {
                        startingPose = Optional
                                .of(startingPose.get().transformBy(Constants.kTurretToRobotCenter.inverse()));
                        robotContainer.getDriveSubsystem()
                                .resetOdometry(startingPose.get());
                    }
                }
                allianceColor = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance() : allianceColor;
                desiredMode = latestDesiredMode;
                startingLocation = latestStartingLocation;
                priorityMidlineSequence = latestPriorityMidlineSequence;
                firstAction = latestFirstAction;
                lastAction = latestLastAction;
                blockedNotes = latestBlockedNotes;
                Logger.recordOutput("Regenerating Auto Command Timestamp", Timer.getFPGATimestamp());
                Logger.recordOutput("Regenerating Auto Command", autonomousCommand.getName());
            }

            // Hood zero
            if (robotContainer.getHood().getCurrentPositionRotations() < 0.0) {
                robotContainer.getHood().resetZeroPoint();
            }
        }
        mIter++;

        if (hasBeenEnabledTeleop && (Timer.getFPGATimestamp() - timeOfLastSync) >= 10.0) {
            OSUtil.fsSyncAsync();
            timeOfLastSync = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void disabledExit() {
        disabledCommand.cancel();
    }

    @Override
    public void autonomousInit() {
        robotContainer.setAutoDefaultCommands();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        mHasBeenEnabled = true;
        robotContainer.getRobotState().setAutoStartTime(Timer.getFPGATimestamp());
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        robotContainer.setTeleopDefaultCommands();
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        boolean validMegatag = (Timer.getFPGATimestamp()
                - robotContainer.getRobotState().lastUsedMegatagTimestamp()) < 3.0;
        if (!mHasBeenEnabled && DriverStation.getAlliance().isPresent() && !validMegatag) {
            var alliance = DriverStation.getAlliance().get();
            robotContainer.getDriveSubsystem().resetOdometry(
                    MathHelpers.pose2dFromRotation(new Rotation2d(Math.toRadians(
                            alliance == Alliance.Red ? 180.0 : 0.0))));
        }
        mHasBeenEnabled = true;
        hasBeenEnabledTeleop = true;
        hasSetNoOp = false;
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.getRobotState().logControllerMode();
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
