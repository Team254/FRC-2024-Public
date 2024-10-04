package com.team254.frc2024.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The {@code IDriveControlBoard} interface defines the method for interacting
 * with the {@link MainDriveControlBoard} and {@link GamepadDriveControlBoard}.
 * It provides a set of methods to access various joystick inputs involved with
 * driving and core button triggers on the controllers.
 */
public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    Trigger resetGyro();

    // Real Controls
    Trigger intake();

    Trigger exhaust();

    Trigger shoot();
}