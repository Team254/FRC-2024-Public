package com.team254.frc2024.subsystems.shooterStage1;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The {@code ShooterSensorIO} interface defines the input/output operations for
 * the shooter sensor system. It is responsible for reading sensor inputs,
 * specifically for detecting whether the shooter has a game piece using a
 * banner sensor.
 */
public interface ShooterSensorIO {
    @AutoLog
    class ShooterSensorInputs {
        public boolean shooterBannerHasPiece;
    }

    default void readInputs(ShooterSensorIO.ShooterSensorInputs inputs) {
    }

    DigitalInput getBanner();
}
