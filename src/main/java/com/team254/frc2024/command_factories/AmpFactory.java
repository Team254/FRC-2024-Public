package com.team254.frc2024.command_factories;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotContainer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;

/**
 * Factory class for creating commands related to the amp subsystem.
 *
 * <p>
 * This class provides a collection of static methods to create various commands
 * for controlling
 * the amp subsystem, which includes operations like intaking, staging, feeding,
 * and scoring with
 * the robot's amp mechanism and elevator. Each method returns a command that
 * can be scheduled in
 * the robot's command-based framework.
 */

public class AmpFactory {

    /**
     * Moves the elevator to the height for source intaking and runs the amp
     * surfaces in reverse to grab the note
     * Exits: when interrupted
     */
    public static Command intakeFromSource(RobotContainer container) {
        return container.getAmp().dutyCycleCommand(() -> Constants.AmpConstants.kAmpIntakeFromSourceDutyCycle)
                .alongWith(container.getElevator()
                        .motionMagicSetpointCommand(() -> Constants.ElevatorConstants.kIntakeFromSourceHeightInches))
                .withName("Amp: Intake from source");
    }

    public static Command feedFromAmp(RobotContainer container) {
        return container.getAmp().dutyCycleCommand(() -> Constants.AmpConstants.kAmpIntakeFromSourceDutyCycle)
                .withName("Amp: Feed from amp");
    }

    public static Command stageNoteBeforeLoweredAmpChopsticks(RobotContainer container) {
        return new SequentialCommandGroup(container.getAmp()
                .runUntilBannerGetsNote(() -> Constants.AmpConstants.kAmpExhaustToStageDutyCycle),
                container.getAmp().dutyCycleCommand(() -> 0.0).withTimeout(0.01));
    }

    public static Command moveStagedNoteToTrapPosition(RobotContainer container) {
        return new SequentialCommandGroup(
                container.getAmp().runUntilBannerGetsNote(() -> 1.0),
                container.getAmp()
                        .spinRotationsBlocking(Constants.AmpConstants.kTrapChopsticksStageRotations,
                                Constants.AmpConstants.kAmpChopsticksStageRotationsTolerance),
                container.getAmp().runUntilBannerLosesNote(() -> Constants.AmpConstants.kAmpSlowlyStageDutyCycle),
                container.getAmp()
                        .spinRotationsBlocking(Constants.AmpConstants.kAmpChopsticksGoBackRotations,
                                Constants.AmpConstants.kAmpChopsticksGoBackRotationsTolerance))
                .withName("Amp: Move to trap position");
    }

    /**
     * Moves elevator to amp scroing height
     * Exits: when elevator is at amp scoring height
     */
    public static Command moveElevatorToAmpScoreHeight(RobotContainer container) {
        return container.getElevator().motionMagicSetpointCommandBlocking(
                Constants.ElevatorConstants.kAmpScoringHeightInches,
                Constants.ElevatorConstants.kElevatorPositioningToleranceInches)
                .withName("Amp: Move elevator to amp score height");
    }

    /**
     * Moves elevator to home
     * Exits: when elevator is at home
     */
    public static Command moveElevatorToHome(RobotContainer container) {
        return container.getElevator().motionMagicSetpointCommandBlocking(
                Constants.ElevatorConstants.kElevatorHomeHeightInches,
                Constants.ElevatorConstants.kElevatorPositioningToleranceInches)
                .withName("Amp: Move elevator to home");
    }

    public static Command stageElevatorToScoreTrap(RobotContainer container) {
        return container.getElevator().motionMagicSetpointCommandBlocking(
                Constants.ElevatorConstants.kClimbHeightInches,
                Constants.ElevatorConstants.kElevatorPositioningToleranceInches)
                .withName("Amp: Move elevator to trap score height");
    }

    public static Command ampShootPulsing(RobotContainer container) {
        // Wave characteristics
        final int periodMs = 100;
        final double amplitude = -1.0;

        // Motor consts
        return container.getAmp().dutyCycleCommand(() -> {
            int ms = (int) Math.round(Timer.getFPGATimestamp() * 1000.0);
            if (ms % periodMs < periodMs / 2) {
                return 0.0;
            } else {
                return amplitude;
            }
        });
    }
}
