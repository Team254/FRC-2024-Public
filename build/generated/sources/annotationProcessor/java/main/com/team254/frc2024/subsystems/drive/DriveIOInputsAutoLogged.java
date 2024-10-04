package com.team254.frc2024.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DriveIOInputsAutoLogged extends DriveIO.DriveIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("GyroAngle", gyroAngle);
  }

  @Override
  public void fromLog(LogTable table) {
    gyroAngle = table.get("GyroAngle", gyroAngle);
  }

  public DriveIOInputsAutoLogged clone() {
    DriveIOInputsAutoLogged copy = new DriveIOInputsAutoLogged();
    copy.gyroAngle = this.gyroAngle;
    return copy;
  }
}
