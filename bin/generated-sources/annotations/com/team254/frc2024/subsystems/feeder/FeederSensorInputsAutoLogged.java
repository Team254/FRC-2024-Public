package com.team254.frc2024.subsystems.feeder;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FeederSensorInputsAutoLogged extends FeederSensorIO.FeederSensorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PizzaBoxBannerHasPiece", pizzaBoxBannerHasPiece);
  }

  @Override
  public void fromLog(LogTable table) {
    pizzaBoxBannerHasPiece = table.get("PizzaBoxBannerHasPiece", pizzaBoxBannerHasPiece);
  }

  public FeederSensorInputsAutoLogged clone() {
    FeederSensorInputsAutoLogged copy = new FeederSensorInputsAutoLogged();
    copy.pizzaBoxBannerHasPiece = this.pizzaBoxBannerHasPiece;
    return copy;
  }
}
