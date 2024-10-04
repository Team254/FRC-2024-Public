package com.team254.frc2024.subsystems.intake;

import com.team254.frc2024.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Hardware implementation of the IntakeSensorIO interface for the intake
 * subsystem.
 * This class interacts with the physical intake banner sensor to detect the
 * presence of game pieces
 * and provides methods to read sensor inputs and retrieve the sensor object.
 */
public class IntakeSensorIOHardware implements IntakeSensorIO {

    protected final DigitalInput intakeBanner;

    public IntakeSensorIOHardware() {
        intakeBanner = new DigitalInput(Constants.SensorConstants.kIntakeBannerSensorPort);
    }

    @Override
    public void readInputs(IntakeSensorInputs inputs) {
        inputs.intakeBannerHasPiece = intakeBanner.get();
    }

    @Override
    public DigitalInput getIntakeBanner() {
        return intakeBanner;
    }
}
