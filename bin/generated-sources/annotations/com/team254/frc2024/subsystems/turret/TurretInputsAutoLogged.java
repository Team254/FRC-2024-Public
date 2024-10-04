package com.team254.frc2024.subsystems.turret;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TurretInputsAutoLogged extends TurretIO.TurretInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentStatorAmps", currentStatorAmps);
    table.put("CurrentSupplyAmps", currentSupplyAmps);
    table.put("Cancoder1AbsolutePosition", cancoder1AbsolutePosition);
    table.put("Cancoder2AbsolutePosition", cancoder2AbsolutePosition);
  }

  @Override
  public void fromLog(LogTable table) {
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentStatorAmps = table.get("CurrentStatorAmps", currentStatorAmps);
    currentSupplyAmps = table.get("CurrentSupplyAmps", currentSupplyAmps);
    cancoder1AbsolutePosition = table.get("Cancoder1AbsolutePosition", cancoder1AbsolutePosition);
    cancoder2AbsolutePosition = table.get("Cancoder2AbsolutePosition", cancoder2AbsolutePosition);
  }

  public TurretInputsAutoLogged clone() {
    TurretInputsAutoLogged copy = new TurretInputsAutoLogged();
    copy.appliedVolts = this.appliedVolts;
    copy.currentStatorAmps = this.currentStatorAmps;
    copy.currentSupplyAmps = this.currentSupplyAmps;
    copy.cancoder1AbsolutePosition = this.cancoder1AbsolutePosition;
    copy.cancoder2AbsolutePosition = this.cancoder2AbsolutePosition;
    return copy;
  }
}
