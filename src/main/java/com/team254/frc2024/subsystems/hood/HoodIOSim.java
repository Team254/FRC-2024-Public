package com.team254.frc2024.subsystems.hood;

import com.team254.frc2024.Constants;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation implementation of the HoodIOHardware class for the hood subsystem.
 * This class simulates the behavior of the hood mechanism using a DCMotorSim to
 * model the motor
 * and hood dynamics. It updates the simulated state of the hood in real-time,
 * applying friction
 * effects and logging key simulation data for analysis.
 */

public class HoodIOSim extends HoodIOHardware {
    protected DCMotorSim mechanismSim = new DCMotorSim(
            DCMotor.getKrakenX60Foc(1),
            1.0 / Constants.HoodConstants.kHoodGearRatio,
            0.1266);
    private Notifier simNotifier = null;

    private double lastUpdateTimestamp;

    public HoodIOSim() {
        simNotifier = new Notifier(() -> {
            updateSimState();
        });
        simNotifier.startPeriodic(0.005);
    }

    @Override
    public void readInputs(HoodInputs inputs) {
        super.readInputs(inputs);
    }

    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

    public void updateSimState() {
        var simState = hoodMotor.getSimState();
        simState.setSupplyVoltage(12.0);
        double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

        mechanismSim.setInput(simVoltage);
        Logger.recordOutput("Hood/Sim/SimulatorVoltage", simVoltage);

        double timestamp = RobotTime.getTimestampSeconds();
        mechanismSim.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        double simPositionRads = mechanismSim.getAngularPositionRad();
        Logger.recordOutput("Hood/Sim/SimulatorPositionRadians", simPositionRads);

        double rotorPosition = Units.radiansToRotations(simPositionRads) / Constants.HoodConstants.kHoodGearRatio;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput("Hood/Sim/setRawRotorPosition", rotorPosition);

        double rotorVel = Units.radiansToRotations(mechanismSim.getAngularVelocityRadPerSec())
                / Constants.HoodConstants.kHoodGearRatio;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput("Hood/Sim/SimulatorVelocityRadS", mechanismSim.getAngularVelocityRadPerSec());
    }
}
