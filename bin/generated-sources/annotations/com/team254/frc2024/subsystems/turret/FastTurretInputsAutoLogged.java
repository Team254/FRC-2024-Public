package com.team254.frc2024.subsystems.turret;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FastTurretInputsAutoLogged extends TurretIO.FastTurretInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TurretPositionAbsolute", turretPositionAbsolute);
    table.put("PositionRad", positionRad);
    table.put("VelocityRadPerSec", velocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    turretPositionAbsolute = table.get("TurretPositionAbsolute", turretPositionAbsolute);
    positionRad = table.get("PositionRad", positionRad);
    velocityRadPerSec = table.get("VelocityRadPerSec", velocityRadPerSec);
  }

  public FastTurretInputsAutoLogged clone() {
    FastTurretInputsAutoLogged copy = new FastTurretInputsAutoLogged();
    copy.turretPositionAbsolute = this.turretPositionAbsolute;
    copy.positionRad = this.positionRad;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    return copy;
  }
}
