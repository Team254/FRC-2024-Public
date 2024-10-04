package com.team254.frc2024.controlboard;

import com.team254.frc2024.Constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The {@code GamepadButtonControlBoard} class provides an interface for
 * managing and interacting
 * with Xbox-style gamepad controllers. It implements the singleton pattern
 * to ensure that only one instance of the control board exists during runtime.
 * <p>
 * This class handles the initialization of gamepad controllers based on
 * configuration constants
 * and joystick type detection. It provides methods to access various button
 * triggers and control
 * functionalities for the robot, including intake mechanisms, climbing, and
 * shooting.
 * </p>
 * <p>
 * The {@code GamepadButtonControlBoard} also supports feedback through rumble
 * functionality, allowing
 * the controller to provide haptic feedback to the user.
 * </p>
 */

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard instance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (instance == null) {
            instance = new GamepadButtonControlBoard();
        }

        return instance;
    }

    private final CommandXboxController controller;
    private final CommandXboxController additionalController;

    private GamepadButtonControlBoard() {
        if (Constants.kForceDriveGamepad || DriverStation.getJoystickIsXbox(Constants.kDriveGamepadPort)) {
            controller = new CommandXboxController(Constants.kDriveGamepadPort);
            additionalController = new CommandXboxController(Constants.kGamepadAdditionalControllerPort);
        } else {
            controller = new CommandXboxController(Constants.kOperatorControllerPort);
        }
    }

    @Override
    public Trigger getIntakeExhaust() {
        return controller.b();
    }

    @Override
    public Trigger shouldIntake() {
        return controller.x();
    }

    @Override
    public Trigger getAmpShot() {
        return controller.y();
    }

    @Override
    public Trigger getSourceIntake() {
        return controller.a();
    }

    @Override
    public Trigger getClimbUp() {
        return controller.povRight();
    }

    @Override
    public Trigger climbArmsDown() {
        return controller.povDown();
    }

    @Override
    public Trigger climbDutyCycleDown() {
        return controller.back().and(controller.start());
    }

    @Override
    public Trigger climbAndScoreTrap() {
        return controller.rightTrigger(0.25);
    }

    @Override
    public Trigger getAimAtGoalMode() {
        return controller.a();
    }

    @Override
    public Trigger getAimAtPoopMode() {
        return controller.y();
    }

    @Override
    public Trigger getAmpMode() {
        return controller.b();
    }

    @Override
    public Trigger getHPMode() {
        return controller.x();
    }

    @Override
    public Trigger getZeroMode() {
        return controller.leftBumper();
    }

    @Override
    public Trigger getClimbMode() {
        return controller.povUpLeft();
    }

    @Override
    public Trigger getSpinUpForShoot() {
        return controller.rightBumper();
    }

    @Override
    public Trigger getExhaustAll() {
        return controller.start().and(controller.leftBumper());
    }

    @Override
    public Trigger getWantTwoNotes() {
        return controller.povUp();
    }

    @Override
    public Trigger getWantPoopShallow() {
        return controller.povLeft();
    }

    @Override
    public Trigger getWantToXWheels() {
        return controller.start().and(controller.back().negate());
    }

    @Override
    public void setRumble(boolean rumble) {
        controller.getHID().setRumble(RumbleType.kBothRumble, rumble ? 1 : 0);
    }

}