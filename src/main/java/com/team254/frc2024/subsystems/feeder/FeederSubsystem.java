package com.team254.frc2024.subsystems.feeder;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team254.frc2024.Constants;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.commands.WaitForDigitalInterruptCommand.Edges;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@code FeederSubsystem} class is responsible for controlling the robot's
 * feeding mechanism
 * using two motors and a sensor. It handles sensor input processing, motor
 * commands, and control loops,
 * allowing the system to detect objects via banners and execute motor commands
 * accordingly.
 * This subsystem incorporates debouncing to mitigate sensor noise and provides
 * several
 * commands for precise motor control based on sensor states.
 */

public class FeederSubsystem extends SubsystemBase {

    public record LeftRightSetpoint(double left, double right) {
    }

    private MotorInputsAutoLogged inputsLeftMotor = new MotorInputsAutoLogged();
    private MotorInputsAutoLogged inputsRightMotor = new MotorInputsAutoLogged();
    private FeederSensorInputsAutoLogged inputsSensors = new FeederSensorInputsAutoLogged();
    private Debouncer bannerDebounce = new Debouncer(Constants.SensorConstants.kFeederDebounceTime,
            Debouncer.DebounceType.kRising);
    private MotorIO ioLeftMotor;
    private MotorIO ioRightMotor;
    private FeederSensorIO ioSensors;

    public FeederSubsystem(ServoMotorSubsystemConfig leftMotorCfg, ServoMotorSubsystemConfig rightMotorCfg,
            final MotorIO ioLeftMotor, final MotorIO ioRightMotor, final FeederSensorIO ioSensors) {
        super("Feeder");
        this.ioLeftMotor = ioLeftMotor;
        this.ioRightMotor = ioRightMotor;
        this.ioSensors = ioSensors;

        setDefaultCommand(defaultCommand());
    }

    @Override
    public void periodic() {
        ioLeftMotor.readInputs(inputsLeftMotor);
        ioRightMotor.readInputs(inputsRightMotor);
        ioSensors.readInputs(inputsSensors);
        Logger.processInputs("Feeder/left", inputsLeftMotor);
        Logger.processInputs("Feeder/right", inputsRightMotor);
        Logger.processInputs("Feeder/sensors", inputsSensors);
    }

    private void setOpenLoopDutyCycleLeftImpl(double dutyCycle) {
        Logger.recordOutput(getName() + "/API/setOpenLoopDutyCycle/dutyCycle/left", dutyCycle);
        ioLeftMotor.setOpenLoopDutyCycle(dutyCycle);
    }

    private void setOpenLoopDutyCycleRightImpl(double dutyCycle) {
        Logger.recordOutput(getName() + "/API/setOpenLoopDutyCycle/dutyCycle/right", dutyCycle);
        ioRightMotor.setOpenLoopDutyCycle(dutyCycle);
    }

    private void setVelocitySetpointLeftImpl(double unitsPerSecond) {
        Logger.recordOutput(getName() + "/API/setVelocitySetpointImpl/UnitsPerS/left", unitsPerSecond);
        ioLeftMotor.setVelocitySetpoint(unitsPerSecond);
    }

    private void setVelocitySetpointRightImpl(double unitsPerSecond) {
        Logger.recordOutput(getName() + "/API/setVelocitySetpointImpl/UnitsPerS/right", unitsPerSecond);
        ioRightMotor.setVelocitySetpoint(unitsPerSecond);
    }

    public Command dutyCycleCommand(DoubleSupplier dutyCycleLeft, DoubleSupplier dutyCycleRight) {
        return startEnd(() -> {
            setOpenLoopDutyCycleLeftImpl(dutyCycleLeft.getAsDouble());
            setOpenLoopDutyCycleRightImpl(dutyCycleRight.getAsDouble());
        }, () -> {
            setOpenLoopDutyCycleLeftImpl(0.0);
            setOpenLoopDutyCycleRightImpl(0.0);
        }).withName(getName() + "DutyCycleControl");
    }

    public Command velocitySetpointCommand(DoubleSupplier velocitySupplierLeft, DoubleSupplier velocitySupplierRight) {
        return startEnd(() -> {
            setVelocitySetpointLeftImpl(velocitySupplierLeft.getAsDouble());
            setVelocitySetpointRightImpl(velocitySupplierRight.getAsDouble());
        }, () -> {
        }).withName(getName() + "VelocityControl");
    }

    public Command velocitySetpointCommand(Supplier<LeftRightSetpoint> velocities) {
        return runEnd(() -> {
            setVelocitySetpointLeftImpl(velocities.get().left);
            setVelocitySetpointRightImpl(velocities.get().right);
        }, () -> {
        }).withName(getName() + "VelocityControl");
    }

    public Command waitForBanner(Edges triggerMode) {
        AtomicBoolean wasBeamBroken = new AtomicBoolean(false);
        return Commands.runOnce(() -> {
            wasBeamBroken.set(ioSensors.getPizzaBanner().get());
        }).andThen(Commands.waitUntil(() -> {
            boolean isBeamBroken = ioSensors.getPizzaBanner().get();
            if (wasBeamBroken.get() && !isBeamBroken) {
                return true;
            }
            wasBeamBroken.set(isBeamBroken);
            return false;
        })).withName("Feeder wait for note to leave");
    }

    public Command defaultCommand() {
        return dutyCycleCommand(() -> 0.0, () -> 0.0).withName(
                "Feeder Default Command Neutral");
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(defaultCommand());
    }

    public Command intakeUnjam(RobotContainer container, DoubleSupplier rpsSupplier) {
        double kTurretZeroHysteresisRange = Math.toRadians(15.0);
        var lastTurretOrientation = new Object() {
            public Optional<Boolean> isPositive = Optional.empty();
        };

        return container.getFeeder().velocitySetpointCommand(() -> {
            double rps = rpsSupplier.getAsDouble();
            LeftRightSetpoint towardsShooter = new LeftRightSetpoint(rps, rps);
            LeftRightSetpoint spinCw = new LeftRightSetpoint(-rps, rps);
            LeftRightSetpoint spinCcw = new LeftRightSetpoint(rps, -rps);

            double turretRotation = container.getRobotState().getLatestRobotToTurret().getValue().getRadians();
            boolean turretIsNearIntake = turretRotation > Math.toRadians(-60.0)
                    && turretRotation < Math.toRadians(60.0);
            boolean noteIsInBackOfPizzaBox = ioSensors.getPizzaBanner().get();
            if (Math.abs(turretRotation) > kTurretZeroHysteresisRange || lastTurretOrientation.isPositive.isEmpty()) {
                lastTurretOrientation.isPositive = Optional.of(turretRotation > 0);
            }

            if (noteIsInBackOfPizzaBox && !turretIsNearIntake) {
                // run in a single direction
                return lastTurretOrientation.isPositive.orElse(true) ? spinCcw : spinCw;
            } else {
                // run towards shooter
                return towardsShooter;
            }
        }).finallyDo(() -> {
            lastTurretOrientation.isPositive = Optional.empty();
        }).withName("Feeder run intake unjamming");
    }

    public boolean hasNoteAtPizzaBoxBanner() {
        return bannerDebounce.calculate(ioSensors.getPizzaBanner().get());
    }
}