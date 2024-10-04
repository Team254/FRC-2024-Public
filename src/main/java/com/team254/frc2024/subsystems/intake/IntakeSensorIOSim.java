package com.team254.frc2024.subsystems.intake;

import edu.wpi.first.hal.simulation.DIODataJNI;

/**
 * Simulation implementation of the IntakeSensorIOHardware for intake subsystem.
 * This class allows for simulating the intake banner sensor in a controlled
 * environment, providing
 * methods to set the simulated sensor state to either detect or not detect a
 * game piece.
 */
public class IntakeSensorIOSim extends IntakeSensorIOHardware {
    public IntakeSensorIOSim() {
        super();
        setNoNote();
    }

    public void setNoNote() {
        DIODataJNI.setValue(this.getIntakeBanner().getChannel(), false);
    }

    public void setHasNote() {
        DIODataJNI.setValue(this.getIntakeBanner().getChannel(), true);
    }
}
