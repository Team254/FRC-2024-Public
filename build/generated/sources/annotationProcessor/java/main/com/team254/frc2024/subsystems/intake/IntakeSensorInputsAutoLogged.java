package com.team254.frc2024.subsystems.intake;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeSensorInputsAutoLogged extends IntakeSensorIO.IntakeSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IntakeBannerHasPiece", intakeBannerHasPiece);
  }

  @Override
  public void fromLog(LogTable table) {
    intakeBannerHasPiece = table.get("IntakeBannerHasPiece", intakeBannerHasPiece);
  }

  public IntakeSensorInputsAutoLogged clone() {
    IntakeSensorInputsAutoLogged copy = new IntakeSensorInputsAutoLogged();
    copy.intakeBannerHasPiece = this.intakeBannerHasPiece;
    return copy;
  }
}
