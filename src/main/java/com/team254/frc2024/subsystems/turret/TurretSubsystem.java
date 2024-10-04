package com.team254.frc2024.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team254.frc2024.RobotState;
import com.team254.lib.loops.IStatusSignalLoop;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and
 * CANCoders
 * to manage turret movement and sensor readings.
 */
public class TurretSubsystem extends SubsystemBase implements IStatusSignalLoop {

    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
    // Fast inputs is used by 250Hz thread. Should not touch without accounting for
    // race conditions.
    private volatile FastTurretInputsAutoLogged fastInputs = new FastTurretInputsAutoLogged();
    // This is used for logging in 50Hz thread.
    private final FastTurretInputsAutoLogged cachedFastInputs = new FastTurretInputsAutoLogged();
    private final TurretIO io;
    private RobotState robotState;
    private double turretPositionSetpointRadiansFromCenter = 0.0;
    private double lastModeChange = 0.0;

    // Constructor
    public TurretSubsystem(final TurretIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;
    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return io.getStatusSignals();
    }

    @Override
    public void onLoop() {
        io.readFastInputs(fastInputs);
        double timestamp = RobotTime.getTimestampSeconds();
        robotState.addTurretUpdates(timestamp, fastInputs.turretPositionAbsolute,
                fastInputs.positionRad,
                fastInputs.velocityRadPerSec);
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.readInputs(inputs);
        Logger.processInputs("Turret", inputs);
        cachedFastInputs.positionRad = getCurrentPosition();
        cachedFastInputs.velocityRadPerSec = robotState.getLatestTurretAngularVelocity();
        cachedFastInputs.turretPositionAbsolute = robotState.getLatestRobotToTurret().getValue();
        Logger.processInputs("Turret/fastInputs", cachedFastInputs);
        Logger.recordOutput("Turret/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    public void updateModeChange() {
        this.lastModeChange = RobotTime.getTimestampSeconds();
    }

    private void setOpenLoopDutyCycleImpl(double dutyCycle) {
        Logger.recordOutput("Turret/API/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
        io.setOpenLoopDutyCycle(dutyCycle);
    }

    public void setPositionSetpointImpl(double radiansFromCenter, double radPerS) {
        Logger.recordOutput("Turret/API/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
        io.setPositionSetpoint(radiansFromCenter, radPerS);
    }

    // Public API
    public Command setOpenLoopDutyCycle(DoubleSupplier dutyCycle) {
        return startEnd(() -> {
            setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
        }, () -> {
        }).withName("Turret DutyCycleControl");
    }

    private double adjustSetpointForWrap(double radiansFromCenter) {
        // We have two options the raw radiansFromCenter or +/- 2 * PI.
        double alternative = radiansFromCenter - 2.0 * Math.PI;
        if (radiansFromCenter < 0.0) {
            alternative = radiansFromCenter + 2.0 * Math.PI;
        }
        if (Math.abs(getCurrentPosition() - alternative) < Math.abs(getCurrentPosition() - radiansFromCenter)) {
            return alternative;
        }
        return radiansFromCenter;
    }

    private boolean unwrapped(double setpoint) {
        // Radians comparison intentional because this is the raw value going into
        // rotor.
        return (RobotTime.getTimestampSeconds() - this.lastModeChange > 0.5) ||
                Util.epsilonEquals(setpoint,
                        getCurrentPosition(), Math.toRadians(10.0));
    }

    private Command positionSetpointUntilUnwrapped(DoubleSupplier radiansFromCenter, DoubleSupplier ffVel) {
        return run(() -> {
            // Intentional do not wrap turret
            double setpoint = radiansFromCenter.getAsDouble();
            setPositionSetpointImpl(setpoint, unwrapped(setpoint) ? ffVel.getAsDouble() : 0.0);
            turretPositionSetpointRadiansFromCenter = setpoint;
        }).until(() -> unwrapped(radiansFromCenter.getAsDouble()));
    }

    // FF is in rad/s.
    public Command positionSetpointCommand(DoubleSupplier radiansFromCenter,
            DoubleSupplier ffVel) {
        return positionSetpointUntilUnwrapped(radiansFromCenter, ffVel).andThen(
                run(() -> {
                    double setpoint = adjustSetpointForWrap(radiansFromCenter.getAsDouble());
                    setPositionSetpointImpl(setpoint, ffVel.getAsDouble());
                    turretPositionSetpointRadiansFromCenter = setpoint;
                })).withName("Turret positionSetpointCommand");
    }

    public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
        return new WaitUntilCommand(() -> {
            return Math.abs(new Rotation2d(getCurrentPosition()).rotateBy(
                    new Rotation2d(radiansFromCenter.getAsDouble()).unaryMinus()).getRadians()) < toleranceRadians;
        }).withName("Turret wait for position");
    }

    public double getSetpoint() {
        return this.turretPositionSetpointRadiansFromCenter;
    }

    public double getCurrentPosition() {
        return robotState.getLatestTurretPositionRadians();
    }

    public void setTeleopDefaultCommand() {
        this.setDefaultCommand(run(() -> {
            setPositionSetpointImpl(turretPositionSetpointRadiansFromCenter, 0.0);
        }).withName("Turret Maintain Setpoint (default)"));
    }
}
