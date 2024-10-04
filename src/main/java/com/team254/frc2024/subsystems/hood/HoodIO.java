package com.team254.frc2024.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

import java.util.Arrays;
import java.util.List;

/**
 * Interface representing the hood subsystem.
 * Defines the structure for interacting with the hood mechanism, including
 * methods for reading input signals and controlling various aspects such as
 * position, neutral mode, and output to the motor.
 */

public interface HoodIO {
    @AutoLog
    class HoodInputs {
        public double positionRad = 0.0;

        public double positionRotations = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentStatorAmps = 0.0;
        public double currentSupplyAmps = 0.0;
    }

    default List<BaseStatusSignal> getStatusSignals() {
        return Arrays.asList();
    }

    default void readInputs(HoodIO.HoodInputs inputs) {
    }

    default void update(final HoodIO.HoodInputs inputs) {
    }

    default void setNeutralMode(NeutralModeValue neutralMode) {
    }

    default void setPositionSetpoint(double radiansFromCenter, double radsPerSec) {
    }

    default void setDutyCycleOut(double percentOutput) {
    }

    void resetZeroPoint();
}
