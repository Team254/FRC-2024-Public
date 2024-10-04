package com.team254.frc2024.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Interface for the intake sensor subsystem of the FRC 2024 robot.
 * This interface defines the structure for interacting with the intake sensor,
 * including methods for reading sensor inputs and retrieving the intake banner
 * sensor.
 */
public interface IntakeSensorIO {
    @AutoLog
    class IntakeSensorInputs {
        public boolean intakeBannerHasPiece;
    }

    default void readInputs(IntakeSensorIO.IntakeSensorInputs inputs) {
    }

    DigitalInput getIntakeBanner();
}
