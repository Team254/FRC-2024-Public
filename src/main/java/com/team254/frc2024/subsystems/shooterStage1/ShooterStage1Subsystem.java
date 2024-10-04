package com.team254.frc2024.subsystems.shooterStage1;

import com.team254.frc2024.Constants;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team254.lib.loops.IStatusSignalLoop;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

/**
 * The {@code ShooterStage1Subsystem} class controls the first stage of the
 * shooter mechanism.
 * It integrates with both motors and sensor inputs, including a banner sensor
 * to detect the presence of game pieces.
 * <p>
 * This subsystem extends the {@link ServoMotorSubsystem}, implementing logic
 * for sensor-based
 * actions such as halting the shooter when a game piece is detected, waiting
 * for a stator current spike,
 * and other key shooting control functionalities.
 * </p>
 * Implements {@link IStatusSignalLoop} to manage motor signals and feedback
 * loops.
 */
public class ShooterStage1Subsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO>
        implements IStatusSignalLoop {
    private ShooterSensorInputsAutoLogged inputsSensors = new ShooterSensorInputsAutoLogged();
    private ShooterSensorIO ioSensors;

    private AtomicBoolean haltRising = new AtomicBoolean(false);
    private AtomicBoolean haltFalling = new AtomicBoolean(false);
    private AtomicBoolean didHalt = new AtomicBoolean(false);
    private Debouncer bannerDebounce = new Debouncer(Constants.SensorConstants.kShooterDebounceTime,
            Debouncer.DebounceType.kRising);

    public ShooterStage1Subsystem(ServoMotorSubsystemConfig c, final MotorIO io, final ShooterSensorIO ioSensors) {
        super(c, new MotorInputsAutoLogged(), io);
        this.ioSensors = ioSensors;
    }

    @Override
    public void periodic() {
        super.periodic();
        ioSensors.readInputs(inputsSensors);
        Logger.processInputs(getName() + "/sensors", inputsSensors);
    }

    public Command waitForStatorCurrentSpike(double ampsToWaitFor) {
        return new WaitUntilCommand(() -> inputs.currentStatorAmps >= ampsToWaitFor);
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Zero shooter RPS"));
    }

    public Command waitForStatorCurrentSpikeDrop(double ampsToWaitFor) {
        return new WaitUntilCommand(() -> inputs.currentStatorAmps <= ampsToWaitFor);
    }

    public Command runUntilBanner(DoubleSupplier velocitySupplier) {
        return Commands.runOnce(() -> {
            didHalt.set(false);
            haltRising.set(true);
        }).andThen(new ConditionalCommand(
                Commands.none(),
                this.velocitySetpointCommand(velocitySupplier).until(didHalt::get),
                this::hasNote).finallyDo(() -> {
                    haltRising.set(false);
                }));
    }

    public Command defaultCommand() {
        return dutyCycleCommand(() -> 0.0);
    }

    public boolean hasNote() {
        return bannerDebounce.calculate(ioSensors.getBanner().get());
    }

    @Override
    public void onLoop() {
        if (haltRising.get() && hasNote()) {
            setVelocitySetpointImpl(0.0);
            haltRising.set(false);
            didHalt.set(true);
        }

        if (haltFalling.get() && !hasNote()) {
            setVelocitySetpointImpl(0.0);
            haltFalling.set(false);
            didHalt.set(true);
        }
    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return new ArrayList<>();
    }
}
