package com.team254.frc2024;

import com.team254.frc2024.commands.autocommands.*;
import com.team254.lib.auto.AutoUtil.FirstAction;
import com.team254.lib.auto.AutoUtil.LastAction;
import com.team254.lib.auto.AutoUtil.PriorityMidlineSequence;
import com.team254.lib.auto.AutoUtil.StartingLocation;

import java.io.File;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Class for selecting auto mode, including checking if a branching auto command
 * is valid.
 */
public class AutoModeSelector {
    public enum DesiredMode {
        TRUSS_SIX_NOTE_AB,
        SPEAKER_SIX_NOTE_AB,
        PRELOAD_BACKOFF,
        SOURCE_TWO_POINT_FIVE_EDC,
        TRUSS_SIX_NOTE_CA,
        TRUSS_SIX_NOTE_BA,
        BRANCHING_AUTO,
        BRANCHING_AUTO_INTERRUPT,
        DO_NOTHING,
    }

    private final RobotContainer container;
    private final LoggedDashboardChooser<DesiredMode> mModeChooser = new LoggedDashboardChooser<>("Auto Routine");
    private final LoggedDashboardChooser<StartingLocation> mStartingLocationChooser = new LoggedDashboardChooser<>(
            "Starting Location");
    private final LoggedDashboardChooser<PriorityMidlineSequence> mPriorityMidlineSequenceChooser = new LoggedDashboardChooser<>(
            "Priority Midline Sequence");
    private final LoggedDashboardChooser<FirstAction> mFirstActionChooser = new LoggedDashboardChooser<>(
            "First Action");
    private final LoggedDashboardChooser<LastAction> mLastActionChooser = new LoggedDashboardChooser<>(
            "Last Action");
    private LoggedDashboardString mBlockedNotes = new LoggedDashboardString("Blocked Midline Notes", "");
    private boolean validCommand = true;

    public AutoModeSelector(RobotContainer container) {
        this.container = container;
        mModeChooser.addDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Branching Auto", DesiredMode.BRANCHING_AUTO);
        mModeChooser.addOption("Branching Auto Interrupt", DesiredMode.BRANCHING_AUTO_INTERRUPT);
        mModeChooser.addOption("(AB) Truss 6 Note", DesiredMode.TRUSS_SIX_NOTE_AB);
        mModeChooser.addOption("(CA) Truss 6 Note", DesiredMode.TRUSS_SIX_NOTE_CA);
        mModeChooser.addOption("(BA) Truss 6 Note", DesiredMode.TRUSS_SIX_NOTE_BA);
        mModeChooser.addOption("(AB) Speaker 6 Note", DesiredMode.SPEAKER_SIX_NOTE_AB);
        mModeChooser.addOption("Preload + Backoff", DesiredMode.PRELOAD_BACKOFF);
        mModeChooser.addOption("(EDC) Source 2.5", DesiredMode.SOURCE_TWO_POINT_FIVE_EDC);

        mPriorityMidlineSequenceChooser.addDefaultOption("ABC", PriorityMidlineSequence.ABC);
        mPriorityMidlineSequenceChooser.addOption("ACB", PriorityMidlineSequence.ACB);
        mPriorityMidlineSequenceChooser.addOption("BAC", PriorityMidlineSequence.BAC);
        mPriorityMidlineSequenceChooser.addOption("BCA", PriorityMidlineSequence.BCA);
        mPriorityMidlineSequenceChooser.addOption("CBA", PriorityMidlineSequence.CBA);
        mPriorityMidlineSequenceChooser.addOption("CAB", PriorityMidlineSequence.CAB);
        mPriorityMidlineSequenceChooser.addOption("EDC", PriorityMidlineSequence.EDC);
        mPriorityMidlineSequenceChooser.addOption("DEC", PriorityMidlineSequence.DEC);

        mStartingLocationChooser.addDefaultOption("Truss", StartingLocation.Truss);
        mStartingLocationChooser.addOption("Speaker", StartingLocation.Speaker);
        mStartingLocationChooser.addOption("Source", StartingLocation.Source);
        mStartingLocationChooser.addOption("Speaker Corner", StartingLocation.SpeakerCorner);
        mStartingLocationChooser.addOption("Amp", StartingLocation.Amp);

        mFirstActionChooser.addDefaultOption("Three Close To Midline", FirstAction.ThreeCloseToMidline);
        mFirstActionChooser.addOption("One Close To Midline", FirstAction.OneCloseToMidline);
        mFirstActionChooser.addOption("Sprint To Midline", FirstAction.SprintToMidline);
        mFirstActionChooser.addOption("Sprint To Midline No Preload", FirstAction.SprintToMidlineNoPreload);

        mLastActionChooser.addDefaultOption("Backoff", LastAction.Backoff);
        mLastActionChooser.addOption("Three Close", LastAction.ThreeClose);
        mLastActionChooser.addOption("Score Preload", LastAction.ScorePreload);
    }

    public LoggedDashboardChooser<DesiredMode> getModeChooser() {
        return mModeChooser;
    }

    public LoggedDashboardString getBlockedMidlineNotes() {
        return mBlockedNotes;
    }

    public LoggedDashboardChooser<PriorityMidlineSequence> getPriorityMidlineSequenceChooser() {
        return mPriorityMidlineSequenceChooser;
    }

    public LoggedDashboardChooser<StartingLocation> getStartingLocationChooser() {
        return mStartingLocationChooser;
    }

    public LoggedDashboardChooser<FirstAction> getFirstActionChooser() {
        return mFirstActionChooser;
    }

    public LoggedDashboardChooser<LastAction> getLastActionChooser() {
        return mLastActionChooser;
    }

    public boolean isValidCommand() {
        return validCommand;
    }

    public Command getAutonomousCommand() {
        if (mModeChooser.get() == null) {
            return Commands.none();
        }
        switch (mModeChooser.get()) {
            case DO_NOTHING:
                return Commands.none();
            case BRANCHING_AUTO:
                break;
            case BRANCHING_AUTO_INTERRUPT:
                break;
            case TRUSS_SIX_NOTE_AB:
                return new TrussSixNoteAB(container);
            case TRUSS_SIX_NOTE_CA:
                return new TrussSixNoteCA(container);
            case TRUSS_SIX_NOTE_BA:
                return new TrussSixNoteBA(container);
            case SPEAKER_SIX_NOTE_AB:
                return new SpeakerSixNoteAB(container);
            case PRELOAD_BACKOFF:
                return new PreloadBackoff(container);
            case SOURCE_TWO_POINT_FIVE_EDC:
                return new SourceTwoPointFiveEDC(container);
            default:
                return Commands.none();
        }
        if (mPriorityMidlineSequenceChooser.get() == null || mStartingLocationChooser.get() == null
                || mFirstActionChooser.get() == null || mLastActionChooser.get() == null
                || mBlockedNotes.get() == null) {
            validCommand = false;
            SmartDashboard.putBoolean("Valid Branching Command", false);
            return Commands.none();
        }
        boolean valid = true;
        if (!(new File(Filesystem.getDeployDirectory() + "/choreo/" + mStartingLocationChooser.get().name() + "-"
                + mFirstActionChooser.get().getName() + "-"
                + mPriorityMidlineSequenceChooser.get().getFirstNote().name() + ".traj").exists())) {
            valid = false;
        }
        for (int i = 0; i < mPriorityMidlineSequenceChooser.get().name().length() - 1; i++) {
            if (!(new File(Filesystem.getDeployDirectory() + "/choreo/"
                    + mPriorityMidlineSequenceChooser.get().name().charAt(i) + "-Score-"
                    + mPriorityMidlineSequenceChooser.get().name().charAt(i + 1) + ".traj").exists())) {
                valid = false;
            }
        }
        if (!valid) {
            validCommand = false;
            SmartDashboard.putBoolean("Valid Branching Command", false);
            return Commands.none();
        }
        validCommand = true;
        SmartDashboard.putBoolean("Valid Branching Command", true);
        if (mModeChooser.get() == DesiredMode.BRANCHING_AUTO) {
            return new BranchingAuto(container, mStartingLocationChooser.get(), mFirstActionChooser.get(),
                    mPriorityMidlineSequenceChooser.get(), mLastActionChooser.get(), mBlockedNotes.get());
        } else if (mModeChooser.get() == DesiredMode.BRANCHING_AUTO_INTERRUPT) {
            return new BranchingAutoInterrupt(container, mStartingLocationChooser.get(), mFirstActionChooser.get(),
                    mPriorityMidlineSequenceChooser.get(), mLastActionChooser.get(), mBlockedNotes.get());
        }
        return Commands.none();
    }

}
