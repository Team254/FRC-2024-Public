package com.team254.frc2024.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Interface representing the I/O operations for the Feeder banner sensor
 * <p>
 * This interface defines methods to read inputs from the Feeder banner sensor
 * and
 * retrieve the sensor inputs for further usage in state management of the note
 * location.
 */

public interface FeederSensorIO {
    @AutoLog
    class FeederSensorInputs {
        public boolean pizzaBoxBannerHasPiece;
    }

    default void readInputs(FeederSensorIO.FeederSensorInputs inputs) {
    }

    DigitalInput getPizzaBanner();
}
