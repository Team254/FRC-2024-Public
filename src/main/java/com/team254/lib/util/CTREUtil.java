package com.team254.lib.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public class CTREUtil {
    public static final int MAX_RETRIES = 10;

    public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
        final int max_num_retries = 10;
        StatusCode statusCode = StatusCode.OK;
        for (int i = 0; i < max_num_retries; ++i) {
            statusCode = function.get();
            if (statusCode == StatusCode.OK)
                break;
        }
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(
                    "Error calling " + function + " on ctre device id " + deviceId + ": " + statusCode, true);
        }
        return statusCode;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
    }

    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }
}
