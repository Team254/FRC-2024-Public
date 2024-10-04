package com.team254.frc2024.subsystems.shooterStage1;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterSensorInputsAutoLogged extends ShooterSensorIO.ShooterSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ShooterBannerHasPiece", shooterBannerHasPiece);
  }

  @Override
  public void fromLog(LogTable table) {
    shooterBannerHasPiece = table.get("ShooterBannerHasPiece", shooterBannerHasPiece);
  }

  public ShooterSensorInputsAutoLogged clone() {
    ShooterSensorInputsAutoLogged copy = new ShooterSensorInputsAutoLogged();
    copy.shooterBannerHasPiece = this.shooterBannerHasPiece;
    return copy;
  }
}
