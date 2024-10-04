package com.team254.frc2024.subsystems.feeder;

import com.team254.frc2024.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class implements the FeederSensorIO interface
 * and creates a Digital Input to get sensor readings.
 */
public class FeederSensorIOHardware implements FeederSensorIO {

    protected final DigitalInput pizzaBoxBanner;

    public FeederSensorIOHardware() {
        pizzaBoxBanner = new DigitalInput(Constants.SensorConstants.kFeederBannerSensorPort);
    }

    @Override
    public void readInputs(FeederSensorInputs inputs) {
        inputs.pizzaBoxBannerHasPiece = pizzaBoxBanner.get();
    }

    @Override
    public DigitalInput getPizzaBanner() {
        return pizzaBoxBanner;
    }
}
