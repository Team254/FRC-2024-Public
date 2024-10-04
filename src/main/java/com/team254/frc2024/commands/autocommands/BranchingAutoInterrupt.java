package com.team254.frc2024.commands.autocommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.commands.WhileCommand;
import com.team254.lib.auto.AutoUtil.FirstAction;
import com.team254.lib.auto.AutoUtil.LastAction;
import com.team254.lib.auto.AutoUtil.PriorityMidlineSequence;
import com.team254.lib.auto.AutoUtil.StartingLocation;
import com.team254.lib.time.RobotTime;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Auto mode that branches based on the intake banner sensor triggering.
 * Initializes all commands, and adds the first command as the first action.
 * Then, based on intake banner sensor, robot note switches to nearest note or
 * scores note and goes to next note in priority sequence. Interrupts the
 * current path if intake banner is triggered early.
 * 
 * See {@link PathFollowingAutoModeCommandGroup} for more details.
 */
public class BranchingAutoInterrupt extends PathFollowingAutoModeCommandGroup {

    public BranchingAutoInterrupt(RobotContainer container, StartingLocation startingLocation,
            FirstAction firstAction,
            PriorityMidlineSequence priorityMidlineSequence, LastAction lastAction, String blockedNotes) {
        super(container,
                startingLocation.name() + "-" + firstAction.getName() + "-"
                        + priorityMidlineSequence.getFirstNote().name(),
                true, priorityMidlineSequence, startingLocation, firstAction, lastAction, blockedNotes);
        initializeNoteSwitchCommands();
        initializeNoteSwitchCWCommands();
        initializeNoteSwitchCCWCommands();
        initializeNoteCWSwitchCWCommands();
        initializeNoteCCWSwitchCCWCommands();
        initializeNoteToScoreToNoteCommands();
        initializeNoteToScoreCommands();
        initializeNoteToThreeCloseCommands();
        initializeNoteToThreeClosePreloadCommands();
        initializeNoteToScorePreloadCommands();
        initializeNoteSwitchMap();
        initializeNoteToScoreToNoteMap();

        switch (lastAction) {
            case ThreeClose:
                switch (firstAction) {
                    case SprintToMidlineNoPreload:
                        midlineNoteThreshold = 2;
                        break;
                    default:
                        midlineNoteThreshold = 3;
                        break;
                }
                break;
            case ScorePreload:
                break;
            case Backoff:
                break;
            default:
                break;
        }
        Logger.recordOutput("Auto/midlineNoteThreshold", midlineNoteThreshold);
        BooleanSupplier ifIntakeBannerDidNotTriggerRecently = () -> {
            return (!(RobotTime.getTimestampSeconds()
                    - container.getRobotState().lastTriggeredIntakeSensorTimestamp() < 1)
                    && (attemptedMidlineNoteCount < midlineNoteThreshold));
        };
        BooleanSupplier ifIntakeBannerTriggeredRecentlyDuringPathCancel = () -> {
            var value = RobotTime.getTimestampSeconds()
                    - container.getRobotState().lastTriggeredIntakeSensorTimestamp() < 1
                    && container.getRobotState().getPathCancel();
            Logger.recordOutput("Auto/ifIntakeBannerTriggeredRecentlyDuringPathCancel", value);
            return value;
        };
        DoubleSupplier timeSinceAutoStart = () -> (Timer.getFPGATimestamp()
                - container.getRobotState().getAutoStartTime());
        addCommands(getStartingLocationToNoteCommand(startingLocation, firstAction).until(
                ifIntakeBannerTriggeredRecentlyDuringPathCancel));
        Command command = (new WhileCommand(
                getClosestNoteSwitchCommand().handleInterrupt(() -> updateNoteAfterSwitchCancel())
                        .until(
                                ifIntakeBannerTriggeredRecentlyDuringPathCancel),
                ifIntakeBannerDidNotTriggerRecently).andThen(
                        getNoteToScoreToNoteCommand(timeSinceAutoStart).until(
                                ifIntakeBannerTriggeredRecentlyDuringPathCancel)))
                .repeatedly();
        addCommands(command);
    }
}
