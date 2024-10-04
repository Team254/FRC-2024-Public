package com.team254.frc2024.commands.autocommands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.team254.frc2024.Constants;
import com.team254.frc2024.Robot;
import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.command_factories.AimFactory;
import com.team254.frc2024.command_factories.IntakeFactory;
import com.team254.frc2024.command_factories.SuperstructureFactory;
import com.team254.frc2024.simulation.SimulatedRobotState;
import com.team254.lib.auto.AutoUtil;
import com.team254.lib.auto.AutoUtil.FirstAction;
import com.team254.lib.auto.AutoUtil.LastAction;
import com.team254.lib.auto.AutoUtil.MidlineNote;
import com.team254.lib.auto.AutoUtil.MidlineNoteRobotOrientation;
import com.team254.lib.auto.AutoUtil.PriorityMidlineSequence;
import com.team254.lib.auto.AutoUtil.StartingLocation;
import com.team254.lib.ctre.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.MathHelpers;
import com.team254.lib.util.ShooterSetpoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * Extension of SequentialCommandGroup that adds a series of autonomous commands
 * based on the selected criteria and the current state of the robot.
 */
public class PathFollowingAutoModeCommandGroup extends SequentialCommandGroup {

    protected final RobotContainer container;
    protected final ArrayList<PathPlannerPath> paths;

    protected final ArrayList<Command> driveCommands;

    protected PathPlannerPath[][] noteSwitchPaths;
    protected PathPlannerPath[][] noteCWSwitchCWPaths;
    protected PathPlannerPath[][] noteCCWSwitchCCWPaths;
    protected PathPlannerPath[][] noteSwitchCCWPaths;
    protected PathPlannerPath[][] noteSwitchCWPaths;
    protected PathPlannerPath[][] noteScoreToNotePaths;
    protected PathPlannerPath[] noteScorePaths;
    protected PathPlannerPath[] noteToThreeClosePaths;
    protected PathPlannerPath[] noteToThreeClosePreloadPaths;
    protected PathPlannerPath[] noteToScorePreloadPaths;

    protected List<Command>[] trussThreeToNoteCommands;
    protected List<Command>[] trussOneToNoteCommands;
    protected List<Command>[] trussSprintToNoteCommands;

    protected List<Command>[] speakerThreeToNoteCommands;
    protected List<Command>[] speakerOneToNoteCommands;

    protected List<Command>[] sourceSprintToNoteCommands;

    protected List<Command>[] speakerCornerSprintToNoteCommands;
    protected List<Command>[] ampSprintToNoteNoPreloadCommands;
    protected List<Command>[] speakerCornerSprintToNoteNoPreloadCommands;

    protected final Pose2d startingPose;

    MidlineNote goingToNote = MidlineNote.B;
    MidlineNote lastGoingToNote;
    MidlineNoteRobotOrientation goingToNoteRobotOrientation = MidlineNoteRobotOrientation.Straight;
    List<MidlineNote> remainingMidlineNotes = new ArrayList<>(
            List.of(MidlineNote.A, MidlineNote.B, MidlineNote.C, MidlineNote.D, MidlineNote.E));
    PriorityMidlineSequence priorityMidlineSequence = PriorityMidlineSequence.ABC;
    StartingLocation startingLocation = StartingLocation.Truss;
    FirstAction firstAction = FirstAction.ThreeCloseToMidline;
    LastAction lastAction = LastAction.ThreeClose;
    int attemptedMidlineNoteCount = 0;
    int midlineNoteThreshold = 6;

    Map<MidlineNote, List<MidlineNote>> closestNoteSwitchMap = Collections.emptyMap();
    Map<MidlineNote, List<MidlineNote>> closestNoteToScoreToNoteMap = Collections.emptyMap();

    protected boolean disableAllCommands = false;

    ApplyChassisSpeeds commandZeroVelocity = new ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0));

    /**
     * Constructor for PathFollowingAutoModeCommandGroup.
     * 
     * @param container            The robot container.
     * @param choreoPathNames      The names of the choreo paths to follow, in
     *                             order.
     * @param useSplitTrajectories Whether to use split trajectories.
     */
    public PathFollowingAutoModeCommandGroup(RobotContainer container, List<String> choreoPathNames,
            boolean useSplitTrajectories) {
        this.container = container;
        paths = new ArrayList<>();
        driveCommands = new ArrayList<>();
        Pose2d tmpStartingPose = null;
        for (String choreoPathName : choreoPathNames) {
            paths.addAll(AutoUtil.getPathPlannerPathsFromChoreoPathName(choreoPathName, useSplitTrajectories));
            if (tmpStartingPose == null && !paths.isEmpty()) {
                PathPlannerPath firstPath = container.getRobotState().isRedAlliance() ? paths.get(0).flipPath()
                        : paths.get(0);
                PathPlannerTrajectory firstTrajectory = firstPath.getTrajectory(new ChassisSpeeds(),
                        MathHelpers.kRotation2dZero);
                tmpStartingPose = firstTrajectory.getInitialTargetHolonomicPose();
            }
        }
        driveCommands.addAll(AutoUtil.getPathPlannerSwerveCommandsFromPathPlannerPaths(paths));
        startingPose = tmpStartingPose;

    }

    /**
     * Constructor for PathFollowingAutoModeCommandGroup.
     * 
     * @param container            The robot container.
     * @param choreoPathName       The name of the choreo path to follow.
     * @param useSplitTrajectories Whether to use split trajectories.
     */
    public PathFollowingAutoModeCommandGroup(RobotContainer container, String choreoPathName,
            boolean useSplitTrajectories) {
        this(container, List.of(choreoPathName), useSplitTrajectories);
    }

    /**
     * Constructor for PathFollowingAutoModeCommandGroup.
     * 
     * @param container               The robot container.
     * @param choreoPathName          The name of the first choreo path to follow.
     * @param useSplitTrajectories    Whether to use split trajectories.
     * @param priorityMidlineSequence The priority midline sequence.
     * @param startingLocation        The starting location.
     * @param firstAction             The first action.
     */
    public PathFollowingAutoModeCommandGroup(RobotContainer container, String choreoPathName,
            boolean useSplitTrajectories, PriorityMidlineSequence priorityMidlineSequence,
            StartingLocation startingLocation, FirstAction firstAction) {
        this(container, List.of(choreoPathName), useSplitTrajectories);
        this.goingToNote = priorityMidlineSequence.getFirstNote();
        this.goingToNoteRobotOrientation = MidlineNoteRobotOrientation.Straight;
        this.priorityMidlineSequence = priorityMidlineSequence;
        this.firstAction = firstAction;
        this.startingLocation = startingLocation;
        if (!Robot.isReal()) {
            if (firstAction == FirstAction.SprintToMidlineNoPreload) {
                container.getSimulatedRobotState().setNoteState(
                        SimulatedRobotState.NoteState.NO_NOTE);
            } else {
                container.getSimulatedRobotState().setNoteState(
                        SimulatedRobotState.NoteState.NOTE_IN_STAGE_1);
            }
        }
        switch (startingLocation) {
            case Truss:
                switch (firstAction) {
                    case ThreeCloseToMidline:
                        initializeTrussThreeToNoteCommands();
                        break;
                    case OneCloseToMidline:
                        initializeTrussOneToNoteCommands();
                        break;
                    case SprintToMidline:
                        initializeTrussSprintToNoteCommands();
                        break;
                    default:
                        break;
                }
            case Speaker:
                switch (firstAction) {
                    case ThreeCloseToMidline:
                        initializeSpeakerThreeToNoteCommands();
                        break;
                    case OneCloseToMidline:
                        initializeSpeakerOneToNoteCommands();
                        break;
                    default:
                        break;
                }
            case Source:
                switch (firstAction) {
                    case SprintToMidline:
                        initializeSourceSprintToNoteCommands();
                        break;
                    default:
                        break;
                }
            case SpeakerCorner:
                switch (firstAction) {
                    case SprintToMidline:
                        initializeSpeakerCornerSprintToNoteCommands();
                        break;
                    case SprintToMidlineNoPreload:
                        initializeSpeakerCornerSprintToNoteNoPreloadCommands();
                        break;
                    default:
                        break;
                }
            case Amp:
                switch (firstAction) {
                    case SprintToMidlineNoPreload:
                        initializeAmpSprintToNoteNoPreloadCommands();
                        break;
                    default:
                        break;
                }
            default:
                break;
        }
    }

    public PathFollowingAutoModeCommandGroup(RobotContainer container, String choreoPathName,
            boolean useSplitTrajectories, PriorityMidlineSequence priorityMidlineSequence,
            StartingLocation startingLocation, FirstAction firstAction, String blockedNotes) {
        this(container, choreoPathName, useSplitTrajectories, priorityMidlineSequence, startingLocation, firstAction);
        for (int i = 0; i < remainingMidlineNotes.size(); i++) {
            if (blockedNotes.contains(remainingMidlineNotes.get(i).name())) {
                remainingMidlineNotes.remove(i);
                i--;
            }
        }
    }

    public PathFollowingAutoModeCommandGroup(RobotContainer container, String choreoPathName,
            boolean useSplitTrajectories, PriorityMidlineSequence priorityMidlineSequence,
            StartingLocation startingLocation, FirstAction firstAction, LastAction lastAction, String blockedNotes) {
        this(container, choreoPathName, useSplitTrajectories, priorityMidlineSequence, startingLocation, firstAction,
                blockedNotes);
        this.lastAction = lastAction;
    }

    protected Command updateNote(MidlineNote goingToNote, MidlineNoteRobotOrientation goingToNoteRobotOrientation) {
        return Commands.runOnce(() -> {
            this.lastGoingToNote = this.goingToNote;
            this.goingToNote = goingToNote;
            this.goingToNoteRobotOrientation = goingToNoteRobotOrientation;
            remainingMidlineNotes.remove(goingToNote);
            attemptedMidlineNoteCount++;
            Logger.recordOutput("Auto/noteState", "updateNote");
            Logger.recordOutput("Auto/RemainingMidlineNotes", remainingMidlineNotes.toString());
            Logger.recordOutput("Auto/goingToNote", this.goingToNote);
            Logger.recordOutput("Auto/lastGoingToNote", this.lastGoingToNote);
            Logger.recordOutput("Auto/attemptedMidlineNoteCount", this.attemptedMidlineNoteCount);
            Logger.recordOutput("Auto/goingToNoteRobotOrientation", this.goingToNoteRobotOrientation);
        });
    }

    protected void updateNoteAfterSwitchCancel() {
        if (container.getRobotState().getLatestFieldToRobot().getValue().getTranslation()
                .getDistance(Constants.kMidlineNoteTranslations.get(goingToNote)) > container.getRobotState()
                        .getLatestFieldToRobot().getValue().getTranslation()
                        .getDistance(Constants.kMidlineNoteTranslations.get(lastGoingToNote))) {
            remainingMidlineNotes.add(goingToNote);
            this.goingToNote = lastGoingToNote;
            attemptedMidlineNoteCount--;
        }
        Logger.recordOutput("Auto/noteState", "updateNoteAfterSwitchCancel");
        Logger.recordOutput("Auto/RemainingMidlineNotes", remainingMidlineNotes.toString());
        Logger.recordOutput("Auto/goingToNote", this.goingToNote);
        Logger.recordOutput("Auto/lastGoingToNote", this.lastGoingToNote);
        Logger.recordOutput("Auto/attemptedMidlineNoteCount", this.attemptedMidlineNoteCount);
        Logger.recordOutput("Auto/goingToNoteRobotOrientation", this.goingToNoteRobotOrientation);
    }

    protected void initializeNoteCWSwitchCWCommands() {
        noteCWSwitchCWPaths = new PathPlannerPath[5][5];
        for (int i = 0; i < noteCWSwitchCWPaths.length; i++) {
            for (int j = 0; j < noteCWSwitchCWPaths[i].length; j++) {
                if (i == j) {
                    noteCWSwitchCWPaths[i][j] = null;
                    continue;
                }
                noteCWSwitchCWPaths[i][j] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                        MidlineNote.fromIndex(i).name() + "CW-Switch-" + MidlineNote.fromIndex(j).name() + "CW");
            }
        }
    }

    protected void initializeNoteCCWSwitchCCWCommands() {
        noteCCWSwitchCCWPaths = new PathPlannerPath[5][5];
        for (int i = 0; i < noteCCWSwitchCCWPaths.length; i++) {
            for (int j = 0; j < noteCCWSwitchCCWPaths[i].length; j++) {
                if (i == j) {
                    noteCCWSwitchCCWPaths[i][j] = null;
                    continue;
                }
                noteCCWSwitchCCWPaths[i][j] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                        MidlineNote.fromIndex(i).name() + "CCW-Switch-" + MidlineNote.fromIndex(j).name() + "CCW");
            }
        }
    }

    protected void initializeNoteSwitchCCWCommands() {
        noteSwitchCCWPaths = new PathPlannerPath[5][5];
        for (int i = 0; i < noteSwitchCCWPaths.length; i++) {
            for (int j = 0; j < noteSwitchCCWPaths[i].length; j++) {
                if (i == j) {
                    noteSwitchCCWPaths[i][j] = null;
                    continue;
                }
                noteSwitchCCWPaths[i][j] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                        MidlineNote.fromIndex(i).name() + "-Switch-" + MidlineNote.fromIndex(j).name() + "CCW");
            }
        }
    }

    protected void initializeNoteSwitchCWCommands() {
        noteSwitchCWPaths = new PathPlannerPath[5][5];
        for (int i = 0; i < noteSwitchCWPaths.length; i++) {
            for (int j = 0; j < noteSwitchCWPaths[i].length; j++) {
                if (i == j) {
                    noteSwitchCWPaths[i][j] = null;
                    continue;
                }
                noteSwitchCWPaths[i][j] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                        MidlineNote.fromIndex(i).name() + "-Switch-" + MidlineNote.fromIndex(j).name() + "CW");
            }
        }
    }

    protected void initializeNoteSwitchCommands() {
        noteSwitchPaths = new PathPlannerPath[5][5];
        for (int i = 0; i < noteSwitchPaths.length; i++) {
            for (int j = 0; j < noteSwitchPaths[i].length; j++) {
                if (i == j) {
                    noteSwitchPaths[i][j] = null;
                    continue;
                }
                noteSwitchPaths[i][j] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                        MidlineNote.fromIndex(i).name() + "-Switch-" + MidlineNote.fromIndex(j).name());
            }
        }
    }

    protected void initializeNoteToScoreToNoteCommands() {
        noteScoreToNotePaths = new PathPlannerPath[5][5];
        for (int i = 0; i < noteScoreToNotePaths.length; i++) {
            for (int j = 0; j < noteScoreToNotePaths[i].length; j++) {
                if (i == j) {
                    noteScoreToNotePaths[i][j] = null;
                    continue;
                }
                noteScoreToNotePaths[i][j] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                        MidlineNote.fromIndex(i).name() + "-Score-" + MidlineNote.fromIndex(j).name());
            }
        }
    }

    protected void initializeNoteToScoreCommands() {
        noteScorePaths = new PathPlannerPath[5];
        for (int i = 0; i < noteScorePaths.length; i++) {
            noteScorePaths[i] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                    MidlineNote.fromIndex(i).name() + "-Score");
        }
    }

    protected void initializeNoteToThreeCloseCommands() {
        noteToThreeClosePaths = new PathPlannerPath[5];
        for (int i = 0; i < noteToThreeClosePaths.length; i++) {
            noteToThreeClosePaths[i] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                    MidlineNote.fromIndex(i).name() + "-ThreeClose");
        }
    }

    protected void initializeNoteToThreeClosePreloadCommands() {
        noteToThreeClosePreloadPaths = new PathPlannerPath[5];
        for (int i = 0; i < noteToThreeClosePreloadPaths.length; i++) {
            noteToThreeClosePreloadPaths[i] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                    MidlineNote.fromIndex(i).name() + "-ThreeClosePreload");
        }
    }

    protected void initializeNoteToScorePreloadCommands() {
        noteToScorePreloadPaths = new PathPlannerPath[5];
        for (int i = 0; i < noteToScorePreloadPaths.length; i++) {
            noteToScorePreloadPaths[i] = AutoUtil.getPathPlannerPathFromChoreoPathName(
                    MidlineNote.fromIndex(i).name() + "-ScorePreload");
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeTrussThreeToNoteCommands() {
        trussThreeToNoteCommands = new List[5];
        for (int i = 0; i < trussThreeToNoteCommands.length; i++) {
            trussThreeToNoteCommands[i] = AutoUtil
                    .getPathPlannerSwerveCommandsFromChoreoPathName("Truss-3-" + MidlineNote.fromIndex(i).name(), true);
            trussThreeToNoteCommands[i].set(trussThreeToNoteCommands[i].size() - 1, trussThreeToNoteCommands[i]
                    .get(trussThreeToNoteCommands[i].size() - 1)
                    .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeTrussOneToNoteCommands() {
        trussOneToNoteCommands = new List[5];
        for (int i = 0; i < trussOneToNoteCommands.length; i++) {
            trussOneToNoteCommands[i] = AutoUtil
                    .getPathPlannerSwerveCommandsFromChoreoPathName("Truss-1-" + MidlineNote.fromIndex(i).name(), true);
            trussOneToNoteCommands[i].set(trussOneToNoteCommands[i].size() - 1, trussOneToNoteCommands[i]
                    .get(trussOneToNoteCommands[i].size() - 1)
                    .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeTrussSprintToNoteCommands() {
        trussSprintToNoteCommands = new List[5];
        for (int i = 0; i < trussSprintToNoteCommands.length; i++) {
            trussSprintToNoteCommands[i] = AutoUtil.getPathPlannerSwerveCommandsFromChoreoPathName(
                    "Truss-Sprint-" + MidlineNote.fromIndex(i).name(), true);
            trussSprintToNoteCommands[i].set(trussSprintToNoteCommands[i].size() - 1, trussSprintToNoteCommands[i]
                    .get(trussSprintToNoteCommands[i].size() - 1)
                    .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeSpeakerThreeToNoteCommands() {
        speakerThreeToNoteCommands = new List[5];
        for (int i = 0; i < speakerThreeToNoteCommands.length; i++) {
            speakerThreeToNoteCommands[i] = AutoUtil.getPathPlannerSwerveCommandsFromChoreoPathName(
                    "Speaker-3-" + MidlineNote.fromIndex(i).name(), true);
            speakerThreeToNoteCommands[i].set(speakerThreeToNoteCommands[i].size() - 1, speakerThreeToNoteCommands[i]
                    .get(speakerThreeToNoteCommands[i].size() - 1)
                    .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeSpeakerOneToNoteCommands() {
        speakerOneToNoteCommands = new List[5];
        for (int i = 0; i < speakerOneToNoteCommands.length; i++) {
            speakerOneToNoteCommands[i] = AutoUtil.getPathPlannerSwerveCommandsFromChoreoPathName(
                    "Speaker-1-" + MidlineNote.fromIndex(i).name(), true);
            speakerOneToNoteCommands[i].set(speakerOneToNoteCommands[i].size() - 1, speakerOneToNoteCommands[i]
                    .get(speakerOneToNoteCommands[i].size() - 1)
                    .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeSourceSprintToNoteCommands() {
        sourceSprintToNoteCommands = new List[5];
        for (int i = 0; i < sourceSprintToNoteCommands.length; i++) {
            sourceSprintToNoteCommands[i] = AutoUtil.getPathPlannerSwerveCommandsFromChoreoPathName(
                    "Source-Sprint-" + MidlineNote.fromIndex(i).name(), true);
            sourceSprintToNoteCommands[i].set(sourceSprintToNoteCommands[i].size() - 1, sourceSprintToNoteCommands[i]
                    .get(sourceSprintToNoteCommands[i].size() - 1)
                    .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeSpeakerCornerSprintToNoteCommands() {
        speakerCornerSprintToNoteCommands = new List[5];
        for (int i = 0; i < speakerCornerSprintToNoteCommands.length; i++) {
            speakerCornerSprintToNoteCommands[i] = AutoUtil.getPathPlannerSwerveCommandsFromChoreoPathName(
                    "SpeakerCorner-Sprint-" + MidlineNote.fromIndex(i).name(), true);
            speakerCornerSprintToNoteCommands[i].set(speakerCornerSprintToNoteCommands[i].size() - 1,
                    speakerCornerSprintToNoteCommands[i]
                            .get(speakerCornerSprintToNoteCommands[i].size() - 1)
                            .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeAmpSprintToNoteNoPreloadCommands() {
        ampSprintToNoteNoPreloadCommands = new List[5];
        for (int i = 0; i < ampSprintToNoteNoPreloadCommands.length; i++) {
            ampSprintToNoteNoPreloadCommands[i] = AutoUtil.getPathPlannerSwerveCommandsFromChoreoPathName(
                    "Amp-NoPreloadSprint-" + MidlineNote.fromIndex(i).name(), true);
            ampSprintToNoteNoPreloadCommands[i].set(ampSprintToNoteNoPreloadCommands[i].size() - 1,
                    ampSprintToNoteNoPreloadCommands[i]
                            .get(ampSprintToNoteNoPreloadCommands[i].size() - 1)
                            .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    @SuppressWarnings("unchecked")
    private void initializeSpeakerCornerSprintToNoteNoPreloadCommands() {
        speakerCornerSprintToNoteNoPreloadCommands = new List[5];
        for (int i = 0; i < speakerCornerSprintToNoteNoPreloadCommands.length; i++) {
            speakerCornerSprintToNoteNoPreloadCommands[i] = AutoUtil.getPathPlannerSwerveCommandsFromChoreoPathName(
                    "SpeakerCorner-NoPreloadSprint-" + MidlineNote.fromIndex(i).name(), true);
            speakerCornerSprintToNoteNoPreloadCommands[i].set(speakerCornerSprintToNoteNoPreloadCommands[i].size() - 1,
                    speakerCornerSprintToNoteNoPreloadCommands[i]
                            .get(speakerCornerSprintToNoteNoPreloadCommands[i].size() - 1)
                            .alongWith(updateNote(MidlineNote.fromIndex(i), MidlineNoteRobotOrientation.Straight)));
        }
    }

    protected void initializeNoteSwitchMap() {
        closestNoteSwitchMap = new HashMap<>();
        closestNoteSwitchMap.put(MidlineNote.A, List.of(MidlineNote.B, MidlineNote.C));
        closestNoteSwitchMap.put(MidlineNote.B, List.of(MidlineNote.A, MidlineNote.C));
        closestNoteSwitchMap.put(MidlineNote.C, List.of(MidlineNote.B, MidlineNote.D));
        closestNoteSwitchMap.put(MidlineNote.D, List.of(MidlineNote.E, MidlineNote.C));
        closestNoteSwitchMap.put(MidlineNote.E, List.of(MidlineNote.D, MidlineNote.C));
    }

    protected void initializeNoteToScoreToNoteMap() {
        closestNoteToScoreToNoteMap = Constants.AutoConstants.kClosestNoteToScoreToNoteMap.get(priorityMidlineSequence);
    }

    public Command getStartingLocationToNoteCommand(StartingLocation startingLocation, FirstAction firstAction) {
        SequentialCommandGroup commands = new SequentialCommandGroup();
        Supplier<ShooterSetpoint> speakerGoalSupplier = ShooterSetpoint
                .speakerSetpointSupplier(container.getRobotState());
        switch (startingLocation) {
            case Truss:
                switch (firstAction) {
                    case SprintToMidline:
                        commands.addCommands(
                                new ParallelCommandGroup(
                                        new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier))
                                                .andThen(
                                                        new ParallelDeadlineGroup(
                                                                new WaitUntilCommand(() -> container.getShooterStage1()
                                                                        .hasNote())
                                                                        .andThen(new WaitUntilCommand(
                                                                                () -> !container.getShooterStage1()
                                                                                        .hasNote())),
                                                                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(
                                                                        container, speakerGoalSupplier))),
                                        trussSprintToNoteCommands[goingToNote.getIndex()]
                                                .get(0)));
                        commands.addCommands(trussSprintToNoteCommands[goingToNote.getIndex()].get(1));
                        break;
                    case OneCloseToMidline:
                        commands.addCommands(
                                new ParallelCommandGroup(
                                        new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier))
                                                .andThen(
                                                        new ParallelDeadlineGroup(
                                                                new WaitUntilCommand(() -> container.getShooterStage1()
                                                                        .hasNote())
                                                                        .andThen(new WaitUntilCommand(
                                                                                () -> !container.getShooterStage1()
                                                                                        .hasNote())),
                                                                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(
                                                                        container, speakerGoalSupplier))),
                                        trussOneToNoteCommands[goingToNote.getIndex()]
                                                .get(0)));
                        commands.addCommands(trussOneToNoteCommands[goingToNote.getIndex()].get(1));
                        commands.addCommands(trussOneToNoteCommands[goingToNote.getIndex()].get(2));
                        break;
                    case ThreeCloseToMidline:
                        commands.addCommands(
                                new ParallelCommandGroup(
                                        new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier))
                                                .andThen(
                                                        new ParallelDeadlineGroup(
                                                                new WaitUntilCommand(() -> container.getShooterStage1()
                                                                        .hasNote())
                                                                        .andThen(new WaitUntilCommand(
                                                                                () -> !container.getShooterStage1()
                                                                                        .hasNote())),
                                                                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(
                                                                        container, speakerGoalSupplier))),
                                        trussThreeToNoteCommands[goingToNote.getIndex()]
                                                .get(0)));
                        commands.addCommands(trussThreeToNoteCommands[goingToNote.getIndex()].get(1));
                        commands.addCommands(trussThreeToNoteCommands[goingToNote.getIndex()].get(2));
                        break;
                    default:
                        break;
                }
                break;
            case Speaker:
                switch (firstAction) {
                    case ThreeCloseToMidline:
                        commands.addCommands(speakerThreeToNoteCommands[goingToNote.getIndex()].get(0));
                        commands.addCommands(speakerThreeToNoteCommands[goingToNote.getIndex()].get(1));
                        break;
                    case OneCloseToMidline:
                        commands.addCommands(
                                new ParallelCommandGroup(
                                        new WaitUntilCommand(() -> AimFactory.onTarget(container, speakerGoalSupplier))
                                                .andThen(
                                                        new ParallelDeadlineGroup(
                                                                new WaitUntilCommand(() -> container.getShooterStage1()
                                                                        .hasNote())
                                                                        .andThen(new WaitUntilCommand(
                                                                                () -> !container.getShooterStage1()
                                                                                        .hasNote())),
                                                                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(
                                                                        container, speakerGoalSupplier))),
                                        speakerOneToNoteCommands[goingToNote.getIndex()].get(0)));
                        commands.addCommands(speakerOneToNoteCommands[goingToNote.getIndex()].get(1));
                        break;
                    default:
                        break;
                }
                break;
            case Source:
                switch (firstAction) {
                    case SprintToMidline:
                        commands.addCommands(sourceSprintToNoteCommands[goingToNote.getIndex()].get(0));
                        break;
                    default:
                        break;
                }
                break;
            case SpeakerCorner:
                switch (firstAction) {
                    case SprintToMidline:
                        commands.addCommands(speakerCornerSprintToNoteCommands[goingToNote.getIndex()].get(0));
                        break;
                    case SprintToMidlineNoPreload:
                        commands.addCommands(speakerCornerSprintToNoteNoPreloadCommands[goingToNote.getIndex()].get(0));
                        break;
                    default:
                        break;
                }
                break;
            case Amp:
                switch (firstAction) {
                    case SprintToMidlineNoPreload:
                        commands.addCommands(ampSprintToNoteNoPreloadCommands[goingToNote.getIndex()].get(0));
                        break;
                    default:
                        break;
                }
                break;
        }

        return commands;
    }

    public Command getClosestNoteSwitchCommand() {
        return new DeferredCommand(() -> {
            Command command = Commands.runOnce(() -> container.getRobotState().disablePathCancel());
            Command command2 = container.getDriveSubsystem().applyRequest(() -> commandZeroVelocity);
            double startTime = RobotTime.getTimestampSeconds();
            if (disableAllCommands) {
                System.out.println("Deferred Note Switch End Time:" + (Timer.getFPGATimestamp() - startTime));
                return command.andThen(command2);
            }
            for (MidlineNote note : closestNoteSwitchMap.get(goingToNote)) {
                if (remainingMidlineNotes.contains(note)) {
                    MidlineNoteRobotOrientation noteRobotOrientation = MidlineNoteRobotOrientation.Straight;
                    if (goingToNote.getIndex() < note.getIndex()) {
                        if (goingToNoteRobotOrientation == MidlineNoteRobotOrientation.Straight) {
                            noteRobotOrientation = MidlineNoteRobotOrientation.CW;
                            Command updateNote = updateNote(note, noteRobotOrientation);
                            command2 = AutoUtil
                                    .getPathPlannerSwerveCommandWithNoteUpdateFromPathPlannerPath(
                                            noteSwitchCWPaths[goingToNote.getIndex()][note.getIndex()], updateNote);
                        } else if (goingToNoteRobotOrientation == MidlineNoteRobotOrientation.CW) {
                            noteRobotOrientation = MidlineNoteRobotOrientation.CW;
                            Command updateNote = updateNote(note, noteRobotOrientation);
                            command2 = AutoUtil
                                    .getPathPlannerSwerveCommandWithNoteUpdateFromPathPlannerPath(
                                            noteCWSwitchCWPaths[goingToNote.getIndex()][note.getIndex()], updateNote);
                        } else {
                            continue;
                        }
                    } else {
                        if (goingToNoteRobotOrientation == MidlineNoteRobotOrientation.Straight) {
                            noteRobotOrientation = MidlineNoteRobotOrientation.CCW;
                            Command updateNote = updateNote(note, noteRobotOrientation);
                            command2 = AutoUtil
                                    .getPathPlannerSwerveCommandWithNoteUpdateFromPathPlannerPath(
                                            noteSwitchCCWPaths[goingToNote.getIndex()][note.getIndex()], updateNote);
                        } else if (goingToNoteRobotOrientation == MidlineNoteRobotOrientation.CCW) {
                            noteRobotOrientation = MidlineNoteRobotOrientation.CCW;
                            Command updateNote = updateNote(note, noteRobotOrientation);
                            command2 = AutoUtil
                                    .getPathPlannerSwerveCommandWithNoteUpdateFromPathPlannerPath(
                                            noteCCWSwitchCCWPaths[goingToNote.getIndex()][note.getIndex()], updateNote);
                        } else {
                            continue;
                        }
                    }
                    // command2 = command2
                    // .alongWith(updateNote(note, noteRobotOrientation))
                    // .withName("Note Switch " + goingToNote.toString() + " To " +
                    // note.toString());
                    command = command.andThen(command2);
                    System.out.println("Deferred Note Switch End Time:" + (Timer.getFPGATimestamp() - startTime));
                    return command;
                }
            }
            return command.andThen(command2);
        }, Set.of(container.getDriveSubsystem()));

    }

    public Command getNoteToScoreToNoteCommand(DoubleSupplier timeSinceAutoStart) {
        return new DeferredCommand(() -> {
            Command command = Commands.runOnce(() -> container.getRobotState().disablePathCancel());
            double startTime = RobotTime.getTimestampSeconds();
            if (disableAllCommands) {
                System.out.println("Deferred Note Score Note End Time:" + (Timer.getFPGATimestamp() - startTime));
                return command;
            }
            switch (lastAction) {
                case Backoff:
                    if (timeSinceAutoStart.getAsDouble() >= 10.0 || attemptedMidlineNoteCount >= midlineNoteThreshold) {
                        disableAllCommands = true;
                        var command2 = AutoUtil
                                .getPathPlannerSwerveCommandFromPathPlannerPath(noteScorePaths[goingToNote.getIndex()]);
                        System.out
                                .println("Deferred Note Score Note End Time:" + (Timer.getFPGATimestamp() - startTime));
                        return command.andThen(command2);
                    }
                    break;
                case ThreeClose:
                    Logger.recordOutput("Auto/*attemptedMidlineNoteCount", this.attemptedMidlineNoteCount);
                    Logger.recordOutput("Auto/*midlineNoteThreshold", this.midlineNoteThreshold);
                    if (timeSinceAutoStart.getAsDouble() >= 5.0 || attemptedMidlineNoteCount >= midlineNoteThreshold) {
                        disableAllCommands = true;
                        var speakerSetpointSupplier = ShooterSetpoint
                                .speakerSetpointSupplier(container.getRobotState());
                        var path = (firstAction == FirstAction.SprintToMidlineNoPreload)
                                ? noteToThreeClosePreloadPaths[goingToNote.getIndex()]
                                : noteToThreeClosePaths[goingToNote.getIndex()];
                        var command2 = AutoUtil.getPathPlannerSwerveCommandFromPathPlannerPath(
                                path).andThen(
                                        new ParallelCommandGroup(
                                                new WaitUntilCommand(
                                                        () -> AimFactory.onTarget(container, speakerSetpointSupplier))
                                                        .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(
                                                                container,
                                                                speakerSetpointSupplier)),
                                                IntakeFactory.runIntake(container,
                                                        () -> Constants.IntakeConstants.kIntakeDutyCycleIntake)));
                        System.out
                                .println("Deferred Note Score Note End Time:" + (Timer.getFPGATimestamp() - startTime));
                        return command.andThen(command2);
                    }
                    break;
                case ScorePreload:
                    if (timeSinceAutoStart.getAsDouble() >= 10.0 || attemptedMidlineNoteCount >= midlineNoteThreshold) {
                        disableAllCommands = true;
                        var speakerSetpointSupplier = ShooterSetpoint
                                .speakerSetpointSupplier(container.getRobotState());
                        var command2 = AutoUtil.getPathPlannerSwerveCommandFromPathPlannerPath(
                                noteToScorePreloadPaths[goingToNote.getIndex()]).andThen(
                                        new ParallelCommandGroup(
                                                new WaitUntilCommand(
                                                        () -> AimFactory.onTarget(container, speakerSetpointSupplier))
                                                        .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(
                                                                container,
                                                                speakerSetpointSupplier)),
                                                IntakeFactory.runIntake(container,
                                                        () -> Constants.IntakeConstants.kIntakeDutyCycleIntake)));
                        System.out
                                .println("Deferred Note Score Note End Time:" + (Timer.getFPGATimestamp() - startTime));
                        return command.andThen(command2);
                    }
                    break;
                case ScorePreloadAndThreeClose:
                    if (timeSinceAutoStart.getAsDouble() >= 10.0) {
                        disableAllCommands = true;
                        var speakerSetpointSupplier = ShooterSetpoint
                                .speakerSetpointSupplier(container.getRobotState());
                        var command2 = AutoUtil.getPathPlannerSwerveCommandFromPathPlannerPath(
                                noteToScorePreloadPaths[goingToNote.getIndex()]).andThen(
                                        new ParallelCommandGroup(
                                                new WaitUntilCommand(
                                                        () -> AimFactory.onTarget(container, speakerSetpointSupplier))
                                                        .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(
                                                                container,
                                                                speakerSetpointSupplier)),
                                                IntakeFactory.runIntake(container,
                                                        () -> Constants.IntakeConstants.kIntakeDutyCycleIntake)));
                        System.out
                                .println("Deferred Note Score Note End Time:" + (Timer.getFPGATimestamp() - startTime));
                        return command.andThen(command2);
                    }
                    break;
                default:
                    break;
            }
            for (MidlineNote note : closestNoteToScoreToNoteMap.get(goingToNote)) {
                if (remainingMidlineNotes.contains(note)) {
                    var command2 = AutoUtil.getPathPlannerSwerveCommandFromPathPlannerPath(
                            noteScoreToNotePaths[goingToNote.getIndex()][note.getIndex()])
                            .alongWith(updateNote(note, MidlineNoteRobotOrientation.Straight))
                            .withName("Note Score From " + goingToNote.toString() + " To " + note.toString());
                    System.out.println("Deferred Note Score Note End Time:" + (Timer.getFPGATimestamp() - startTime));
                    return command.andThen(command2);
                }
            }
            disableAllCommands = true;
            return command.andThen(
                    AutoUtil.getPathPlannerSwerveCommandFromPathPlannerPath(noteScorePaths[goingToNote.getIndex()]));
        }, Set.of(container.getDriveSubsystem()));
    }

    public Optional<Pose2d> getStartingPose() {
        return Optional.ofNullable(this.startingPose);
    }
}
