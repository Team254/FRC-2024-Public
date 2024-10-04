package com.team254.frc2024.controlboard;

import com.team254.frc2024.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The {@code ControlBoard} class manages the control inputs for the robot,
 * integrating both driving controls
 * and button controls into a single interface. It implements the
 * {@link IDriveControlBoard} and
 * {@link IButtonControlBoard} interfaces, providing methods for all required
 * controls.
 *
 * <p>
 * This class follows the singleton pattern, ensuring that only one instance of
 * {@code ControlBoard} is
 * created during the runtime. It allows for switching between different control
 * boards (e.g., gamepad or
 * main control board) based on configuration or input type.
 *
 * <p>
 * The {@code ControlBoard} class provides methods to:
 * <ul>
 * <li>Retrieve input values for driving operations like throttle, strafe, and
 * rotation.</li>
 * <li>Access triggers for various robot actions, such as shooting, intake, and
 * turret movement.</li>
 * <li>Control robot modes, including aim mode, climb mode, and more specialized
 * functions.</li>
 * <li>Handle rumble feedback on the controller for tactile feedback during
 * operation.</li>
 * </ul>
 *
 * @see IDriveControlBoard
 * @see IButtonControlBoard
 */

public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {
    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final IDriveControlBoard driveControlBoard;
    private final IButtonControlBoard buttonControlBoard;

    private ControlBoard() {
        boolean useDriveGamepad = Constants.kForceDriveGamepad ||
                DriverStation.getJoystickIsXbox(Constants.kDriveGamepadPort);
        driveControlBoard = useDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        buttonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return driveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return driveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return driveControlBoard.getRotation();
    }

    @Override
    public Trigger exhaust() {
        return driveControlBoard.exhaust();
    }

    @Override
    public Trigger resetGyro() {
        return driveControlBoard.resetGyro();
    }

    @Override
    public Trigger intake() {
        return driveControlBoard.intake();
    }

    @Override
    public Trigger shoot() {
        return driveControlBoard.shoot();
    }

    @Override
    public Trigger getIntakeExhaust() {
        return buttonControlBoard.getIntakeExhaust();
    }

    @Override
    public Trigger shouldIntake() {
        return buttonControlBoard.shouldIntake();
    }

    @Override
    public Trigger getAmpShot() {
        return buttonControlBoard.getAmpShot();
    }

    @Override
    public Trigger getSourceIntake() {
        return buttonControlBoard.getSourceIntake();
    }

    @Override
    public Trigger getClimbUp() {
        return buttonControlBoard.getClimbUp();
    }

    @Override
    public Trigger climbArmsDown() {
        return buttonControlBoard.climbArmsDown();
    }

    @Override
    public Trigger climbDutyCycleDown() {
        return buttonControlBoard.climbDutyCycleDown();
    }

    @Override
    public Trigger climbAndScoreTrap() {
        return buttonControlBoard.climbAndScoreTrap();
    }

    @Override
    public Trigger getAimAtGoalMode() {
        return buttonControlBoard.getAimAtGoalMode();
    }

    @Override
    public Trigger getAimAtPoopMode() {
        return buttonControlBoard.getAimAtPoopMode();
    }

    @Override
    public Trigger getAmpMode() {
        return buttonControlBoard.getAmpMode();
    }

    @Override
    public Trigger getHPMode() {
        return buttonControlBoard.getHPMode();
    }

    @Override
    public Trigger getZeroMode() {
        return buttonControlBoard.getZeroMode();
    }

    @Override
    public Trigger getClimbMode() {
        return buttonControlBoard.getClimbMode();
    }

    @Override
    public Trigger getSpinUpForShoot() {
        return buttonControlBoard.getSpinUpForShoot();
    }

    @Override
    public Trigger getExhaustAll() {
        return buttonControlBoard.getExhaustAll();
    }

    @Override
    public Trigger getWantTwoNotes() {
        return buttonControlBoard.getWantTwoNotes();
    }

    @Override
    public Trigger getWantPoopShallow() {
        return buttonControlBoard.getWantPoopShallow();
    }

    @Override
    public Trigger getWantToXWheels() {
        return buttonControlBoard.getWantToXWheels();
    }

    public void setRumble(boolean rumble) {
        buttonControlBoard.setRumble(rumble);
    }

    public Command rumble() {
        return Commands.startEnd(() -> setRumble(true), () -> setRumble(false));
    }
}