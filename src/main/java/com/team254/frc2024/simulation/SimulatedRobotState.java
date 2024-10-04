package com.team254.frc2024.simulation;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.RobotState;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Modified robot state used in simulation. Includes simulated note states.
 */
public class SimulatedRobotState {
  public enum NoteState {
    NO_NOTE,
    NOTE_IN_STAGE_1,
    NOTE_IN_INTAKE
  }

  TimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth = TimeInterpolatableBuffer
      .createBuffer(RobotState.LOOKBACK_TIME);

  private NoteState noteState = NoteState.NOTE_IN_STAGE_1;
  private LatchedBoolean stage1TurnedOn = new LatchedBoolean();
  private LatchedBoolean intakeOnNearMidlineNote = new LatchedBoolean();

  private RobotContainer container;

  private double latestNoteInIntakeTimestamp = -1.0;

  public SimulatedRobotState(RobotContainer container) {
    this.container = container;
    SmartDashboard.putString("Accepted Midline Notes", "ABCDE");
  }

  synchronized public void addFieldToRobot(Pose2d pose) {
    fieldToRobotSimulatedTruth.addSample(RobotTime.getTimestampSeconds(), pose);
  }

  synchronized public NoteState getNoteState() {
    return this.noteState;
  }

  synchronized public void setNoteState(NoteState state) {
    this.noteState = state;
    stage1TurnedOn.update(false);
  }

  synchronized public Pose2d getLatestFieldToRobot() {
    var entry = fieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
    if (entry == null) {
      return null;
    }
    return entry.getValue();
  }

  synchronized public void updateSim() {
    boolean stage1On = stage1TurnedOn.update(
        container.getShooterStage1().getCurrentVelocity() > 1.0);
    boolean robotNearMidlineNote = false;
    for (var entry : Constants.kMidlineNoteTranslations.entrySet()) {
      var translation = entry.getValue();
      if (container.getRobotState().getLatestFieldToRobot().getValue().getTranslation().getDistance(translation) < 0.5
          && SmartDashboard.getString("Accepted Midline Notes", "ABCDE")
              .contains(entry.getKey().name())) {
        robotNearMidlineNote = true;
      }
    }
    boolean intakeOnNearAllowedMidlineNote = intakeOnNearMidlineNote.update(robotNearMidlineNote);

    switch (noteState) {
      case NOTE_IN_INTAKE -> {
        container.getSimulatedIntakeSensors().setHasNote();
        if (RobotTime.getTimestampSeconds() - latestNoteInIntakeTimestamp >= 0.5) {
          this.noteState = NoteState.NOTE_IN_STAGE_1;
        }
      }
      case NOTE_IN_STAGE_1 -> {
        container.getSimulatedStage1Sensors().setHasNote();
        container.getSimulatedIntakeSensors().setNoNote();
        if (stage1On) {
          // We have fired note or rather jammed it into stage2.
          this.noteState = NoteState.NO_NOTE;
        }
      }
      case NO_NOTE -> {
        container.getSimulatedStage1Sensors().setNoNote();
        container.getSimulatedIntakeSensors().setNoNote();
        if (intakeOnNearAllowedMidlineNote) {
          latestNoteInIntakeTimestamp = RobotTime.getTimestampSeconds();
          this.noteState = NoteState.NOTE_IN_INTAKE;
        }
      }
    }
  }
}
