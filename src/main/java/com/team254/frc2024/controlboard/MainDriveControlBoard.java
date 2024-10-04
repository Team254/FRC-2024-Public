package com.team254.frc2024.controlboard;

import com.team254.frc2024.Constants;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * MainDriveControlBoard class is responsible for managing the control interface
 * for the drive system in the robot. It provides methods to get joystick inputs
 * for throttle, strafe, and rotation, as well as triggers for various robot
 * actions such as resetting the gyro, intake, exhaust, and shooting.
 *
 * <p>
 * This class implements the Singleton design pattern to ensure that only one
 * instance is created.
 *
 * <p>
 * It interacts with joysticks for controlling robot movements and buttons
 * for specific actions and has a deadband defined in {@link Constants}.
 */

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard instance = null;

    public static MainDriveControlBoard getInstance() {
        if (instance == null) {
            instance = new MainDriveControlBoard();
        }

        return instance;
    }

    private final Joystick translateStick;
    private final Joystick rotateStick;

    private MainDriveControlBoard() {
        translateStick = new Joystick(Constants.kMainThrottleJoystickPort);
        rotateStick = new Joystick(Constants.kMainTurnJoystickPort);
    }

    @Override
    public double getThrottle() {
        return Util.handleDeadband(-translateStick.getRawAxis(1), Constants.kDriveJoystickThreshold);
    }

    @Override
    public double getStrafe() {
        return Util.handleDeadband(-translateStick.getRawAxis(0), Constants.kDriveJoystickThreshold);
    }

    @Override
    public double getRotation() {
        return Util.handleDeadband(rotateStick.getRawAxis(0), Constants.kJoystickThreshold);
    }

    @Override
    public Trigger resetGyro() {
        return new JoystickButton(rotateStick, 2);
    }

    @Override
    public Trigger intake() {
        return new JoystickButton(translateStick, 1);
    }

    @Override
    public Trigger exhaust() {
        return new JoystickButton(translateStick, 2);
    }

    @Override
    public Trigger shoot() {
        return new JoystickButton(rotateStick, 1);
    }
}