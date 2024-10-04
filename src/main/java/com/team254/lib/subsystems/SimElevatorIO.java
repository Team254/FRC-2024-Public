package com.team254.lib.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class SimElevatorIO extends TalonFXIO {
    public static class SimElevatorConfig {
        public double gearing;
        public double carriageMass; // in KG
        public double drumRadius;
    }

    protected ElevatorSim sim;
    private Notifier simNotifier = null;
    private double lastUpdateTimestamp = 0.0;

    public SimElevatorIO(ServoMotorSubsystemConfig config,
            SimElevatorConfig elevatorConfig) {
        super(config);

        sim = new ElevatorSim(
                DCMotor.getKrakenX60Foc(1),
                1.0 / elevatorConfig.gearing,
                elevatorConfig.carriageMass,
                elevatorConfig.drumRadius,
                config.kMinPositionUnits,
                config.kMaxPositionUnits,
                true, 0.0);
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

        // Find current state of sim in M
        double simPositionM = sim.getPositionMeters();
        Logger.recordOutput(config.name + "/Sim/SimulatorPositionMeters", simPositionM);

        // Mutate rotor position
        double rotorPosition = Units.metersToInches(simPositionM) / config.unitToRotorRatio;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput(config.name + "/Sim/setRawRotorPosition", rotorPosition);

        // Mutate rotor vel
        double rotorVel = Units.metersToInches(sim.getVelocityMetersPerSecond()) / config.unitToRotorRatio;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput(config.name + "/Sim/SimulatorVelocityMS", sim.getVelocityMetersPerSecond());
    }
}
