package com.team254.frc2024.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.Constants;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import com.team254.lib.time.RobotTime;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@code ShooterStage2Subsystem} class represents the second stage of the
 * shooter system.
 * This subsystem controls the top and bottom motors of the shooter, allowing
 * for control
 * of motor velocities and duty cycles during teleoperated and autonomous modes.
 */

public class ShooterStage2Subsystem extends SubsystemBase {

    private MotorInputsAutoLogged inputsTopMotor = new MotorInputsAutoLogged();
    private MotorInputsAutoLogged inputsBottomMotor = new MotorInputsAutoLogged();

    private MotorIO ioTopMotor;
    private MotorIO ioBottomMotor;

    public ShooterStage2Subsystem(ServoMotorSubsystemConfig bottomConfig, final MotorIO ioBottomMotor,
            ServoMotorSubsystemConfig topConfig, final MotorIO ioTopMotor) {
        super("Shooter");
        this.ioTopMotor = ioTopMotor;
        this.ioBottomMotor = ioBottomMotor;

        setDefaultCommand(defaultCommand());
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        ioBottomMotor.readInputs(inputsBottomMotor);
        ioTopMotor.readInputs(inputsTopMotor);
        Logger.processInputs(getName() + "/bottom", inputsBottomMotor);
        Logger.processInputs(getName() + "/top", inputsTopMotor);

        Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Zero shooter RPS"));
    }

    public Command defaultCommand() {
        return dutyCycleCommand(() -> 0.0).withName(
                getName() + " Default Command Neutral");
    }

    public Command dutyCycleCommand(DoubleSupplier setpoint) {
        return runEnd(() -> {
            setOpenLoopDutyCycleImpl(setpoint.getAsDouble());
        }, () -> {
            setOpenLoopDutyCycleImpl(0.0);
        }).withName(getName() + " Duty Cycle Command");
    }

    public Command velocitySetpointCommand(DoubleSupplier setpoint) {
        return runEnd(() -> {
            double vel = setpoint.getAsDouble();
            setVelocitySetpointImpl(vel);
        }, () -> {
            setOpenLoopDutyCycleImpl(0.0);
        }).withName(getName() + " Velocity Command");
    }

    private void setOpenLoopDutyCycleImpl(double dutyCycle) {
        Logger.recordOutput(getName() + "/top/API/setOpenLoopDutyCycle/dutyCycle",
                dutyCycle * Constants.ShooterConstants.kTopRollerSpeedupFactor);
        Logger.recordOutput(getName() + "/bottom/API/setOpenLoopDutyCycle/dutyCycle",
                dutyCycle * Constants.ShooterConstants.kBottomRollerSpeedupFactor);
        ioTopMotor.setOpenLoopDutyCycle(dutyCycle * Constants.ShooterConstants.kTopRollerSpeedupFactor);
        ioBottomMotor.setOpenLoopDutyCycle(dutyCycle * Constants.ShooterConstants.kBottomRollerSpeedupFactor);
    }

    private void setVelocitySetpointImpl(double unitsPerSecond) {
        Logger.recordOutput(getName() + "/top/API/setVelocitySetpointImpl/UnitsPerS",
                unitsPerSecond * Constants.ShooterConstants.kTopRollerSpeedupFactor);
        Logger.recordOutput(getName() + "/bottom/API/setVelocitySetpointImpl/UnitsPerS",
                unitsPerSecond * Constants.ShooterConstants.kBottomRollerSpeedupFactor);
        ioTopMotor.setVelocitySetpoint(unitsPerSecond * Constants.ShooterConstants.kTopRollerSpeedupFactor);
        ioBottomMotor.setVelocitySetpoint(unitsPerSecond * Constants.ShooterConstants.kBottomRollerSpeedupFactor);
    }

    public double getCurrentVelocity() {
        return ((inputsBottomMotor.velocityUnitsPerSecond / Constants.ShooterConstants.kBottomRollerSpeedupFactor)
                + (inputsTopMotor.velocityUnitsPerSecond / Constants.ShooterConstants.kTopRollerSpeedupFactor)) / 2.0;
    }
}