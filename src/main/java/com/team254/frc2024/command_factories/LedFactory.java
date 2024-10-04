package com.team254.frc2024.command_factories;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.commands.DynamicSelectCommand;
import com.team254.frc2024.subsystems.led.LedState;

import edu.wpi.first.wpilibj2.command.Command;

public class LedFactory {

    private enum GenericShootWithIntakeLEDMode {
        DEFAULT,
        OFF_TARGET_WITH_NOTE,
        ON_TARGET_WITH_NOTE,
        ON_TARGET_NO_NOTE
    }

    private static final double kLowBattery = 12.3;

    public static Command shootAndIntakeLEDs(RobotContainer container, BooleanSupplier onTargetSupplier,
            BooleanSupplier hasNoteSupplier, LedState modeColor) {
        return new DynamicSelectCommand<GenericShootWithIntakeLEDMode>(Map.ofEntries(
                Map.entry(GenericShootWithIntakeLEDMode.DEFAULT, container.getLeds().commandSolidColor(modeColor)),
                Map.entry(GenericShootWithIntakeLEDMode.OFF_TARGET_WITH_NOTE,
                        container.getLeds().commandBlinkingStateWithoutScheduler(modeColor, LedState.kOff, 0.075,
                                0.075)),
                Map.entry(GenericShootWithIntakeLEDMode.ON_TARGET_WITH_NOTE,
                        container.getLeds().commandBlinkingStateWithoutScheduler(modeColor, LedState.kOff, 0.075,
                                0.075)),
                Map.entry(GenericShootWithIntakeLEDMode.ON_TARGET_NO_NOTE,
                        container.getLeds().commandSolidColor(modeColor))),
                () -> {
                    boolean hasNote = hasNoteSupplier.getAsBoolean();
                    boolean onTarget = onTargetSupplier.getAsBoolean();

                    if (hasNote && onTarget) {
                        return GenericShootWithIntakeLEDMode.ON_TARGET_WITH_NOTE;
                    } else if (hasNote && !onTarget) {
                        return GenericShootWithIntakeLEDMode.OFF_TARGET_WITH_NOTE;
                    } else if (!hasNote && onTarget) {
                        return GenericShootWithIntakeLEDMode.ON_TARGET_NO_NOTE;
                    }

                    return GenericShootWithIntakeLEDMode.DEFAULT;
                });
    }

    public static Command speakerModeLEDs(RobotContainer container, BooleanSupplier onTargetSupplier) {
        return shootAndIntakeLEDs(
                container, onTargetSupplier, () -> (container.getIntake().hasNoteAtIntakeBanner()
                        || container.getShooterStage1().hasNote() || container.getFeeder().hasNoteAtPizzaBoxBanner()),
                LedState.kGreen);
    }

    public static Command poopModeLEDs(RobotContainer container, BooleanSupplier onTargetSupplier) {
        return shootAndIntakeLEDs(
                container, onTargetSupplier, () -> (container.getIntake().hasNoteAtIntakeBanner()
                        || container.getShooterStage1().hasNote() || container.getFeeder().hasNoteAtPizzaBoxBanner()),
                LedState.kRed);
    }

    public static Command hpModeLEDs(RobotContainer container, BooleanSupplier onTargetSupplier) {
        return shootAndIntakeLEDs(
                container, onTargetSupplier, () -> (container.getIntake().hasNoteAtIntakeBanner()
                        || container.getShooterStage1().hasNote() || container.getFeeder().hasNoteAtPizzaBoxBanner()),
                LedState.kBlue);
    }

    public static Command ampModeLEDs(RobotContainer container) {
        return shootAndIntakeLEDs(
                container, () -> false, () -> (container.getIntake().hasNoteAtIntakeBanner()
                        || container.getShooterStage1().hasNote() || container.getFeeder().hasNoteAtPizzaBoxBanner()),
                LedState.kWhite);
    }

    public static Command climbModeLEDs(RobotContainer container) {
        return container.getLeds().commandSolidPattern(LedState.kRainbow);
    }

    public static Command twoNotesLEDs(RobotContainer container) {
        return container.getLeds().commandSolidColor(() -> {
            return LedState.kPurple;
        }).withName("Wants two notes purple LEDs");
    }

    public static Command batteryLEDs(RobotContainer container, DoubleSupplier voltageSupplier) {
        return container.getLeds().commandSolidColor(() -> {
            if (voltageSupplier.getAsDouble() < kLowBattery) {
                return LedState.kLowBattery;
            } else {
                return LedState.kGoodBattery;
            }
        });
    }
}
