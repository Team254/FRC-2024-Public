package com.team254.frc2024.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The {@code IButtonControlBoard} interface defines the methods for interacting
 * with {@link GamepadButtonControlBoard}
 * It provides a set of methods to access various button triggers and control
 * functionalities through Xbox-style gamepads.
 * <p>
 * This interface includes methods for controlling
 * mechanisms such as intake, climbing, shooting, and other robot functions.
 * </p>
 */

public interface IButtonControlBoard {

    Trigger getIntakeExhaust();

    Trigger shouldIntake();

    Trigger getAmpShot();

    Trigger getSourceIntake();

    Trigger getClimbUp();

    Trigger climbAndScoreTrap();

    Trigger climbArmsDown();

    Trigger climbDutyCycleDown();

    // Real Controls
    Trigger getAimAtGoalMode();

    Trigger getAimAtPoopMode();

    Trigger getAmpMode();

    Trigger getHPMode();

    Trigger getZeroMode();

    Trigger getClimbMode();

    Trigger getSpinUpForShoot();

    Trigger getExhaustAll();

    Trigger getWantTwoNotes();

    Trigger getWantPoopShallow();

    Trigger getWantToXWheels();

    void setRumble(boolean rumble);
}