package com.team254.frc2024.subsystems.turret;

import com.ctre.phoenix6.sim.ChassisReference;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.Constants;

// Simulation class for TurretIO. This uses the Talon's simulation system, so this class extends
// TurretIOHardware, and can mutate the sim state of the underlying hardware.
public class TurretIOSim extends TurretIOHardware {
    protected DCMotorSim mechanismSim = new DCMotorSim(
            DCMotor.getKrakenX60Foc(1),
            1.0 / Constants.TurretConstants.kTurretGearRatio,
            0.2981858);
    protected double lastUpdateTimestamp;
    private Notifier simNotifier = null;

    // private double lastSimTime;
    public TurretIOSim() {
        // Assume that Turret boots zeroed.
        canCoder1To1.setPosition(0.0);
        canCoder3To1.setPosition(0.0);

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState();
        });
        simNotifier.startPeriodic(0.005);
        canCoder1To1.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        canCoder3To1.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public void readInputs(TurretInputs inputs) {
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
        // Step sim
        var simState = talon.getSimState();
        simState.setSupplyVoltage(12.0);
        double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

        mechanismSim.setInput(simVoltage);
        Logger.recordOutput("Turret/Sim/SimulatorVoltage", simVoltage);

        double timestamp = RobotTime.getTimestampSeconds();
        mechanismSim.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        // Find current state of sim in radians from 0 point
        double simPositionRads = mechanismSim.getAngularPositionRad();
        Logger.recordOutput("Turret/Sim/SimulatorPositionRadians", simPositionRads);

        canCoder1To1.getSimState().setRawPosition(Units.radiansToRotations(simPositionRads));
        canCoder1To1.getSimState().setVelocity(Units.radiansToRotations(mechanismSim.getAngularVelocityRadPerSec()));
        double simTurret1Cancoder = canCoder1To1.getPosition().getValueAsDouble();
        Logger.recordOutput("Turret/Sim/Turret CANcoder 1 Position", simTurret1Cancoder);

        double simTurret2Cancoder = canCoder3To1.getPosition().getValueAsDouble();
        Logger.recordOutput("Turret/Sim/Turret CANcoder 2 Position", simTurret2Cancoder);

        // Mutate rotor position
        double rotorPosition = Units.radiansToRotations(simPositionRads) / Constants.TurretConstants.kTurretGearRatio;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput("Turret/Sim/setRawRotorPosition", rotorPosition);

        // Mutate rotor vel
        double rotorVel = Units.radiansToRotations(mechanismSim.getAngularVelocityRadPerSec())
                / Constants.TurretConstants.kTurretGearRatio;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput("Turret/Sim/SimulatorVelocityRadS", mechanismSim.getAngularVelocityRadPerSec());
    }
}

// Drag MyPose into 3d pose and make into component(robot) for components to be
// visible.