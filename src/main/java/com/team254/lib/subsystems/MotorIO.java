package com.team254.lib.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface MotorIO {
    void readInputs(MotorInputs inputs);

    void setOpenLoopDutyCycle(double dutyCycle);

    // These are in the "units" of the subsystem (rad, m).
    void setPositionSetpoint(double units);

    void setMotionMagicSetpoint(double units);

    void setNeutralMode(NeutralModeValue mode);

    void setVelocitySetpoint(double unitsPerSecond);

    void setCurrentPositionAsZero();

    void setCurrentPosition(double positionUnits);

    void setEnableSoftLimits(boolean forward, boolean reverse);
}
