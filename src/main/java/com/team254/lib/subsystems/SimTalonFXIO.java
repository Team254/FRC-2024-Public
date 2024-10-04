package com.team254.lib.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class SimTalonFXIO extends TalonFXIO {
    protected DCMotorSim sim;
    private Notifier simNotifier = null;
    private double lastUpdateTimestamp = 0.0;

    public SimTalonFXIO(ServoMotorSubsystemConfig config) {
        super(config);

        sim = new DCMotorSim(
                DCMotor.getKrakenX60Foc(1),
                1.0 / config.unitToRotorRatio,
                config.momentOfInertia);
        // Assume that config is correct (which it might not be)
        talon.getSimState().Orientation = (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            updateSimState();
        });
        simNotifier.startPeriodic(0.005);
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

    protected void updateSimState() {
        var simState = talon.getSimState();
        double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

        sim.setInput(simVoltage);
        Logger.recordOutput(config.name + "/Sim/SimulatorVoltage", simVoltage);

        double timestamp = RobotTime.getTimestampSeconds();
        sim.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        // Find current state of sim in radians from 0 point
        double simPositionRads = sim.getAngularPositionRad();
        Logger.recordOutput(config.name + "/Sim/SimulatorPositionRadians", simPositionRads);

        // Mutate rotor position
        double rotorPosition = Units.radiansToRotations(simPositionRads) / config.unitToRotorRatio;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput(config.name + "/Sim/setRawRotorPosition", rotorPosition);

        // Mutate rotor vel
        double rotorVel = Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) / config.unitToRotorRatio;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput(config.name + "/Sim/SimulatorVelocityRadS", sim.getAngularVelocityRadPerSec());
    }

}
