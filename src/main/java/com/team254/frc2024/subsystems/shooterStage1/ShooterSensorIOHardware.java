package com.team254.frc2024.subsystems.shooterStage1;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The {@code ShooterSensorIOHardware} class implements the
 * {@link ShooterSensorIO} interface
 * and provides hardware-level control for the shooter sensor system. It is
 * responsible
 * for managing the digital input connected to the shooter banner sensor, which
 * detects
 * whether the shooter has a game piece.
 */
public class ShooterSensorIOHardware implements ShooterSensorIO {

    protected final DigitalInput shooterBanner;

    public ShooterSensorIOHardware(int dioPort) {
        shooterBanner = new DigitalInput(dioPort);
    }

    @Override
    public void readInputs(ShooterSensorInputs inputs) {
        inputs.shooterBannerHasPiece = shooterBanner.get();
    }

    @Override
    public DigitalInput getBanner() {
        return shooterBanner;
    }
}
