package com.team254.frc2024.subsystems.amp;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AmpSensorInputsAutoLogged extends AmpSensorIO.AmpSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AmpBannerHasPiece", ampBannerHasPiece);
    table.put("AmpPostChopstickBannerHasPiece", ampPostChopstickBannerHasPiece);
  }

  @Override
  public void fromLog(LogTable table) {
    ampBannerHasPiece = table.get("AmpBannerHasPiece", ampBannerHasPiece);
    ampPostChopstickBannerHasPiece = table.get("AmpPostChopstickBannerHasPiece", ampPostChopstickBannerHasPiece);
  }

  public AmpSensorInputsAutoLogged clone() {
    AmpSensorInputsAutoLogged copy = new AmpSensorInputsAutoLogged();
    copy.ampBannerHasPiece = this.ampBannerHasPiece;
    copy.ampPostChopstickBannerHasPiece = this.ampPostChopstickBannerHasPiece;
    return copy;
  }
}
