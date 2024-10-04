package com.team254.frc2024.subsystems.hood;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HoodInputsAutoLogged extends HoodIO.HoodInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionRad", positionRad);
    table.put("PositionRotations", positionRotations);
    table.put("VelocityRadPerSec", velocityRadPerSec);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentStatorAmps", currentStatorAmps);
    table.put("CurrentSupplyAmps", currentSupplyAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    positionRad = table.get("PositionRad", positionRad);
    positionRotations = table.get("PositionRotations", positionRotations);
    velocityRadPerSec = table.get("VelocityRadPerSec", velocityRadPerSec);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentStatorAmps = table.get("CurrentStatorAmps", currentStatorAmps);
    currentSupplyAmps = table.get("CurrentSupplyAmps", currentSupplyAmps);
  }

  public HoodInputsAutoLogged clone() {
    HoodInputsAutoLogged copy = new HoodInputsAutoLogged();
    copy.positionRad = this.positionRad;
    copy.positionRotations = this.positionRotations;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    copy.appliedVolts = this.appliedVolts;
    copy.currentStatorAmps = this.currentStatorAmps;
    copy.currentSupplyAmps = this.currentSupplyAmps;
    return copy;
  }
}
