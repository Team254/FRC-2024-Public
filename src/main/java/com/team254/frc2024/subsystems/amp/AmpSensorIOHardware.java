package com.team254.frc2024.subsystems.amp;

import com.team254.frc2024.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class implements the AmpSensorIO interface
 * and creates DigitalInputs for the two Amp banner
 * sensors and returns their status.
 */

public class AmpSensorIOHardware implements AmpSensorIO {

    protected final DigitalInput ampBanner;
    protected final DigitalInput ampPostChopstickBanner;

    public AmpSensorIOHardware() {
        ampBanner = new DigitalInput(Constants.SensorConstants.kAmpBannerSensorPort);
        ampPostChopstickBanner = new DigitalInput(Constants.SensorConstants.kAmpPostChopstickSensorPort);
    }

    @Override
    public void readInputs(AmpSensorInputs inputs) {
        inputs.ampBannerHasPiece = ampBanner.get();
        inputs.ampPostChopstickBannerHasPiece = ampPostChopstickBanner.get();
    }

    @Override
    public DigitalInput getAmpBanner() {
        return ampBanner;
    }

    @Override
    public DigitalInput getAmpPostChopstickBanner() {
        return ampPostChopstickBanner;
    }
}
