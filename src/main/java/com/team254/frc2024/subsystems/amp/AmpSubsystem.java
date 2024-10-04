package com.team254.frc2024.subsystems.amp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team254.frc2024.Constants;
import com.team254.lib.loops.IStatusSignalLoop;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import com.team254.lib.util.Util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/**
 * The {@code AmpSubsystem} class is responsible for controlling the robot's amp
 * mechanism using a
 * motor and sensors. It handles sensor input processing, motor commands, and
 * control loops,
 * allowing the system to detect objects via banners and execute motor commands
 * accordingly.
 * This subsystem incorporates debouncing to mitigate sensor noise and provides
 * several
 * commands for precise motor control based on sensor states.
 */

public class AmpSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> implements IStatusSignalLoop {
    private AmpSensorInputsAutoLogged inputsSensors = new AmpSensorInputsAutoLogged();
    private AmpSensorIO ioSensors;

    private AtomicBoolean haltRising = new AtomicBoolean(false);
    private AtomicBoolean haltFalling = new AtomicBoolean(false);
    private AtomicBoolean didHalt = new AtomicBoolean(false);
    private Debouncer bannerDebounce = new Debouncer(Constants.SensorConstants.kAmpDebounceTime,
            Debouncer.DebounceType.kRising);

    private AtomicBoolean haltPostChopstickRising = new AtomicBoolean(false);
    private AtomicBoolean haltPostChopstickFalling = new AtomicBoolean(false);
    private AtomicBoolean didPostChopstickHalt = new AtomicBoolean(false);
    private Debouncer bannerPostChopstickDebounce = new Debouncer(Constants.SensorConstants.kAmpDebounceTime,
            Debouncer.DebounceType.kRising);

    public AmpSubsystem(ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO, final AmpSensorIO sensorIO) {
        super(motorConfig, new MotorInputsAutoLogged(), motorIO);
        this.ioSensors = sensorIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        ioSensors.readInputs(inputsSensors);
        inputsSensors.ampBannerHasPiece = getHasNote();
        inputsSensors.ampPostChopstickBannerHasPiece = getHasNotePostChopstick();
        Logger.processInputs(getName() + "/sensors", inputsSensors);
    }

    public Command runUntilPostChopsticBannerGetsNote(DoubleSupplier dutyCycleSupplier) {
        return Commands.runOnce(() -> {
            didPostChopstickHalt.set(false);
            haltPostChopstickRising.set(true);
        }).andThen(new ConditionalCommand(
                Commands.none(),
                this.dutyCycleCommand(dutyCycleSupplier).until(didPostChopstickHalt::get),
                this::getHasNotePostChopstick).finallyDo(() -> {
                    haltPostChopstickRising.set(false);
                }));
    }

    public Command runUntilBannerGetsNote(DoubleSupplier dutyCycleSupplier) {
        return Commands.runOnce(() -> {
            didHalt.set(false);
            haltRising.set(true);
        }).andThen(new ConditionalCommand(
                Commands.none(),
                this.dutyCycleCommand(dutyCycleSupplier).until(didHalt::get),
                this::getHasNote).finallyDo(() -> {
                    haltRising.set(false);
                }));
    }

    public Command runUntilBannerLosesNote(DoubleSupplier dutyCycleSupplier) {
        return Commands.runOnce(() -> {
            didHalt.set(false);
            haltFalling.set(true);
        }).andThen(new ConditionalCommand(
                Commands.none(),
                this.dutyCycleCommand(dutyCycleSupplier).until(didHalt::get),
                this::getDoesNotHaveNote).finallyDo(() -> {
                    haltFalling.set(false);
                }));
    }

    public boolean getHasNote() {
        return bannerDebounce.calculate(ioSensors.getAmpBanner().get());
    }

    public boolean getHasNotePostChopstick() {
        return bannerPostChopstickDebounce.calculate(ioSensors.getAmpPostChopstickBanner().get());
    }

    public boolean getDoesNotHaveNote() {
        return !ioSensors.getAmpBanner().get();
    }

    public Command spinRotationsBlocking(double numberOfRotations, double toleranceRotations) {
        var targetPosition = new Object() {
            double rotations = 0;
        };
        return new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    targetPosition.rotations = inputs.unitPosition + numberOfRotations;
                }),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> Util.epsilonEquals(inputs.unitPosition, targetPosition.rotations,
                                toleranceRotations)),
                        motionMagicSetpointCommand(() -> targetPosition.rotations)))
                .withName("Amp spin for number of rotations (motion magic)");
    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return new ArrayList<>();
    }

    @Override
    public void onLoop() {
        final boolean hasNoteAtBanner = getHasNote();
        final boolean hasNotePostChopsticks = getHasNotePostChopstick();
        if (haltRising.get() && hasNoteAtBanner) {
            setOpenLoopDutyCycleImpl(0.0);
            haltRising.set(false);
            didHalt.set(true);
        }

        if (haltFalling.get() && !hasNoteAtBanner) {
            setOpenLoopDutyCycleImpl(0.0);
            haltFalling.set(false);
            didHalt.set(true);
        }

        if (haltPostChopstickRising.get() && hasNotePostChopsticks) {
            setOpenLoopDutyCycleImpl(0.0);
            haltPostChopstickRising.set(false);
            didPostChopstickHalt.set(true);
        }

        if (haltPostChopstickFalling.get() && !hasNotePostChopsticks) {
            setOpenLoopDutyCycleImpl(0.0);
            haltPostChopstickFalling.set(false);
            didPostChopstickHalt.set(true);
        }
    }
}
