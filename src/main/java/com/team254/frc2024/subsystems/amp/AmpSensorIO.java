package com.team254.frc2024.subsystems.amp;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Interface representing the I/O operations for the Amp sensors.
 * <p>
 * This interface defines methods to read inputs from the Amp beam-break sensors
 * and
 * retrieve the sensor inputs for further usage in state management of the note
 * location.
 */

public interface AmpSensorIO {
    @AutoLog
    class AmpSensorInputs {
        public boolean ampBannerHasPiece;
        public boolean ampPostChopstickBannerHasPiece;
    }

    default void readInputs(AmpSensorIO.AmpSensorInputs inputs) {
    }

    DigitalInput getAmpBanner();

    DigitalInput getAmpPostChopstickBanner();
}
