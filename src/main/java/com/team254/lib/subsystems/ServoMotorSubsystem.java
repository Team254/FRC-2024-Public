package com.team254.lib.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.frc2024.Constants;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj2.command.*;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * RollerMotorSubsystem
 * 
 * @param <T>
 */
public class ServoMotorSubsystem<T extends MotorInputsAutoLogged, U extends MotorIO> extends SubsystemBase {
    protected U io;
    protected T inputs;
    private double positionSetpoint = 0.0;

    protected ServoMotorSubsystemConfig conf;

    public ServoMotorSubsystem(ServoMotorSubsystemConfig config, T inputs, U io) {
        super(config.name);
        this.conf = config;
        this.io = io;
        this.inputs = inputs;

        setDefaultCommand(dutyCycleCommand(() -> 0.0).withName(
                getName() + " Default Command Neutral"));
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.readInputs(inputs);
        Logger.processInputs(getName(), inputs);
        Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
        Logger.recordOutput(getName() + "/API/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
        io.setOpenLoopDutyCycle(dutyCycle);
    }

    protected void setPositionSetpointImpl(double units) {
        positionSetpoint = units;
        Logger.recordOutput(getName() + "/API/setPositionSetpointImp/Units", units);
        io.setPositionSetpoint(units);
    }

    protected void setNeutralModeImpl(NeutralModeValue mode) {
        Logger.recordOutput(getName() + "/API/setNeutralModeImpl/Mode", mode);
        io.setNeutralMode(mode);
    }

    protected void setMotionMagicSetpointImpl(double units) {
        positionSetpoint = units;
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImp/Units", units);
        io.setMotionMagicSetpoint(units);
    }

    protected void setVelocitySetpointImpl(double unitsPerSecond) {
        Logger.recordOutput(getName() + "/API/setVelocitySetpointImpl/UnitsPerS", unitsPerSecond);
        io.setVelocitySetpoint(unitsPerSecond);
    }

    public double getCurrentPosition() {
        return inputs.unitPosition;
    }

    public double getCurrentVelocity() {
        return inputs.velocityUnitsPerSecond;
    }

    public double getPositionSetpoint() {
        return positionSetpoint;
    }

    public Command dutyCycleCommand(DoubleSupplier dutyCycle) {
        return runEnd(() -> {
            setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
        }, () -> {
            setOpenLoopDutyCycleImpl(0.0);
        }).withName(getName() + " DutyCycleControl");
    }

    public Command velocitySetpointCommand(DoubleSupplier velocitySupplier) {
        return runEnd(() -> {
            setVelocitySetpointImpl(velocitySupplier.getAsDouble());
        }, () -> {
        }).withName(getName() + " VelocityControl");
    }

    public Command setCoast() {
        return startEnd(() -> setNeutralModeImpl(NeutralModeValue.Coast),
                () -> setNeutralModeImpl(NeutralModeValue.Brake)).withName(getName() + "CoastMode")
                .ignoringDisable(true);
    }

    public Command positionSetpointCommand(DoubleSupplier unitSupplier) {
        return runEnd(() -> {
            setPositionSetpointImpl(unitSupplier.getAsDouble());
        }, () -> {
        }).withName(getName() + " positionSetpointCommand");
    }

    public Command positionSetpointUntilOnTargetCommand(DoubleSupplier unitSupplier, DoubleSupplier epsilon) {
        return new ParallelDeadlineGroup(new WaitUntilCommand(
                () -> Util.epsilonEquals(unitSupplier.getAsDouble(), inputs.unitPosition, epsilon.getAsDouble())),
                positionSetpointCommand(unitSupplier));
    }

    public Command motionMagicSetpointCommand(DoubleSupplier unitSupplier) {
        return runEnd(() -> {
            setMotionMagicSetpointImpl(unitSupplier.getAsDouble());
        }, () -> {
        }).withName(getName() + " motionMagicSetpointCommand");
    }

    protected void setCurrentPositionAsZero() {
        io.setCurrentPositionAsZero();
    }

    protected void setCurrentPosition(double positionUnits) {
        io.setCurrentPosition(positionUnits);
    }

    public Command waitForElevatorPosition(DoubleSupplier targetPosition) {
        return new WaitUntilCommand(() -> Util.epsilonEquals(inputs.unitPosition, targetPosition.getAsDouble(),
                Constants.ElevatorConstants.kElevatorPositioningToleranceInches));
    }

    protected Command withoutLimitsTemporarily() {
        var prev = new Object() {
            boolean fwd = false;
            boolean rev = false;
        };
        return Commands.startEnd(() -> {
            prev.fwd = conf.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
            prev.rev = conf.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
            io.setEnableSoftLimits(false, false);
        }, () -> {
            io.setEnableSoftLimits(prev.fwd, prev.rev);
        });
    }

}
