package com.team254.frc2024.commands.autocommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
 * scores note and goes to next note in priority sequence.
 * 
 * See {@link PathFollowingAutoModeCommandGroup} for more details.
 */
public class BranchingAuto extends PathFollowingAutoModeCommandGroup {
    public BranchingAuto(RobotContainer container, StartingLocation startingLocation, FirstAction firstAction,
            PriorityMidlineSequence priorityMidlineSequence, LastAction lastAction, String blockedNotes) {
        super(container,
                startingLocation.name() + "-" + firstAction.getName() + "-"
                        + priorityMidlineSequence.getFirstNote().name(),
                true, priorityMidlineSequence, startingLocation, firstAction, lastAction, blockedNotes); // need to
                                                                                                         // provide
                                                                                                         // first choreo
                                                                                                         // path name to
                                                                                                         // reset
                                                                                                         // odometry and
                                                                                                         // first
                                                                                                         // midline note
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
        addCommands(getStartingLocationToNoteCommand(startingLocation, firstAction));
        BooleanSupplier ifIntakeBannerDidNotTriggerRecently = () -> !(RobotTime.getTimestampSeconds()
                - container.getRobotState().lastTriggeredIntakeSensorTimestamp() < 1);
        DoubleSupplier timeSinceAutoStart = () -> (Timer.getFPGATimestamp()
                - container.getRobotState().getAutoStartTime());
        Command command = new WhileCommand(getClosestNoteSwitchCommand(),
                ifIntakeBannerDidNotTriggerRecently).andThen(getNoteToScoreToNoteCommand(timeSinceAutoStart));
        addCommands(command);
        Command command2 = new WhileCommand(getClosestNoteSwitchCommand(),
                ifIntakeBannerDidNotTriggerRecently).andThen(getNoteToScoreToNoteCommand(timeSinceAutoStart));
        addCommands(command2);
        Command command3 = new WhileCommand(getClosestNoteSwitchCommand(),
                ifIntakeBannerDidNotTriggerRecently).andThen(getNoteToScoreToNoteCommand(timeSinceAutoStart));
        addCommands(command3);
        Command command4 = new WhileCommand(getClosestNoteSwitchCommand(),
                ifIntakeBannerDidNotTriggerRecently).andThen(getNoteToScoreToNoteCommand(timeSinceAutoStart));
        addCommands(command4);
    }
}
