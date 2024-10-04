package com.team254.frc2024.subsystems.led;

import java.util.function.Supplier;

import com.team254.frc2024.Constants;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The {@code LedSubsystem} class represents the subsystem responsible for
 * controlling LED patterns and behavior.
 * It utilizes the {@link LedIO} interface to send commands to LED hardware and
 * provides a variety of command-based methods
 * for setting solid colors, patterns, blinking states, and percentage-based LED
 * displays.
 */

public class LedSubsystem extends SubsystemBase {
    private final LedIO io;

    public record PercentageSetpoint(double pct, LedState color) {
    }

    public LedSubsystem(final LedIO io) {
        this.io = io;

        setDefaultCommand(commandSolidColor(LedState.kOff).ignoringDisable(true)
                .withName("LED Default Command"));
    }

    public Command commandSolidColor(LedState state) {
        return run(() -> setSolidColor(state)).ignoringDisable(true).withName("LED Solid Color");
    }

    public Command commandSolidColor(Supplier<LedState> state) {
        return run(() -> setSolidColor(state.get())).ignoringDisable(true)
                .withName("LED Solid Color");
    }

    public Command commandSolidPattern(LedState[] states) {
        return run(() -> setSolidPattern(states)).ignoringDisable(true).withName("LED Solid Pattern");
    }

    public Command commandPercentageFull(double percentageFull, LedState state) {
        return run(() -> setPercentageFull(percentageFull, state)).ignoringDisable(true);
    }

    public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
        return run(() -> setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color))
                .ignoringDisable(true);
    }

    public Command commandBlinkingState(LedState stateOne, LedState stateTwo, double durationOne, double durationTwo) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> setSolidColor(stateOne)),
                new WaitCommand(durationOne),
                Commands.runOnce(() -> setSolidColor(stateTwo)),
                new WaitCommand(durationTwo)).repeatedly().ignoringDisable(true).withName("Blinking LED command");
    }

    public Command commandBlinkingStateWithoutScheduler(LedState stateOne, LedState stateTwo, double durationOne,
            double durationTwo) {
        var state = new Object() {
            public boolean color1 = true;
            public double timestamp = Timer.getFPGATimestamp();
        };
        return Commands.runOnce(() -> {
            state.color1 = true;
            state.timestamp = Timer.getFPGATimestamp();
        }).andThen(commandSolidColor(() -> {
            if (state.color1 && state.timestamp + durationOne <= Timer.getFPGATimestamp()) {
                state.color1 = false;
                state.timestamp = Timer.getFPGATimestamp();
            } else if (!state.color1 && state.timestamp + durationTwo <= Timer.getFPGATimestamp()) {
                state.color1 = true;
                state.timestamp = Timer.getFPGATimestamp();
            }

            if (state.color1) {
                return stateOne;
            } else {
                return stateTwo;
            }
        })).ignoringDisable(true).withName("Blinking LED command");
    }

    public Command commandBlinkingState(LedState stateOne, LedState stateTwo, double duration) {
        return commandBlinkingState(stateOne, stateTwo, duration, duration).ignoringDisable(true);
    }

    private void setSolidColor(LedState state) {
        io.writePixels(state);
    }

    private void setSolidPattern(LedState[] states) {
        io.writePixels(states);
    }

    private void setPercentageFull(double percentageFull, LedState state) {
        LedState[] pixels = new LedState[Constants.LEDConstants.kMaxLEDCount / 2];
        for (int i = 0; i < pixels.length; i++) {
            if (i < pixels.length * Util.limit(percentageFull, 0.0, 1.0)) {
                pixels[i] = state;
            }
        }

        io.writePixels(mirror(pixels));
    }

    private LedState[] mirror(LedState[] pixels) {
        LedState[] fullPixels = new LedState[Constants.LEDConstants.kMaxLEDCount];

        for (int i = Constants.LEDConstants.kCandleLEDCount; i < Constants.LEDConstants.kCandleLEDCount
                + (Constants.LEDConstants.kNonCandleLEDCount / 2); i++) {
            fullPixels[fullPixels.length - i - 1] = pixels[i];
            fullPixels[i] = pixels[i];
        }

        return fullPixels;
    }
}
