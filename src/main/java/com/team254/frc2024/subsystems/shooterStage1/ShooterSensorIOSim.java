package com.team254.frc2024.subsystems.shooterStage1;

import edu.wpi.first.hal.simulation.DIODataJNI;

/**
 * The {@code ShooterSensorIOSim} class extends the
 * {@link ShooterSensorIOHardware}
 * and provides simulation-specific functionality for the shooter sensor system.
 * This class allows for simulating the presence or absence of a game piece.
 */
public class ShooterSensorIOSim extends ShooterSensorIOHardware {
    public ShooterSensorIOSim(int dioPort) {
        super(dioPort);
        setHasNote();
    }

    public void setNoNote() {
        DIODataJNI.setValue(this.getBanner().getChannel(), false);
    }

    public void setHasNote() {
        DIODataJNI.setValue(this.getBanner().getChannel(), true);
    }
}
