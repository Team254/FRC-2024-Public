package com.team254.frc2024.controlboard;

import com.team254.frc2024.Constants;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * GamepadDriveControlBoard class provides an interface for controlling the
 * robot
 * using a gamepad (Xbox controller). It allows for accessing joystick values
 * (throttle, strafe, and rotation) and triggers for various actions such as
 * resetting the gyro, intake, exhaust, and shooting.
 *
 * <p>
 * This class uses the Singleton design pattern, ensuring that only one
 * instance is created.
 *
 * <p>
 * The controls are mapped using the {@link CommandXboxController} class and
 * constants defined in {@link Constants} for the gamepad port.
 */

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard instance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (instance == null) {
            instance = new GamepadDriveControlBoard();
        }

        return instance;
    }

    private final CommandXboxController controller;

    private GamepadDriveControlBoard() {
        controller = new CommandXboxController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return -controller.getLeftY();
    }

    @Override
    public double getStrafe() {
        return -controller.getLeftX();
    }

    @Override
    public double getRotation() {
        return -controller.getRightX();
    }

    @Override
    public Trigger exhaust() {
        return controller.start();
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }

    @Override
    public Trigger intake() {
        return controller.leftTrigger(0.125);
    }

    @Override
    public Trigger shoot() {
        return controller.rightTrigger(0.125);
    }
}