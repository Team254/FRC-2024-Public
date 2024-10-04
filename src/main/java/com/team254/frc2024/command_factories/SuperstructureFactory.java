package com.team254.frc2024.command_factories;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotContainer;
import com.team254.frc2024.commands.VisualizeNoteShot;
import com.team254.lib.util.ShooterSetpoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * The {@code SuperstructureFactory} class is responsible for creating and
 * managing various commands related to the robot's superstructure.
 * It provides factory methods for creating complex sequences of commands for
 * actions like intaking, shooting, and exhausting game pieces.
 *
 * <p>
 * This class handles the coordination of multiple subsystems such as the
 * intake, feeder, shooter, turret, hood, elevator, and amp.
 * It ensures that these subsystems work together seamlessly to perform the
 * desired actions during a match.
 *
 * <p>
 * Some of the key functionalities provided by this class include:
 * <ul>
 * <li>Intaking game pieces and preparing them for shooting.</li>
 * <li>Shooting game pieces with precise control over shooter subsystems.</li>
 * <li>Handling special scenarios such as exhausting game pieces or staging them
 * for later use.</li>
 * <li>Managing subsystem states to achieve specific robot behaviors in various
 * modes of operation.</li>
 * </ul>
 *
 * <p>
 * The methods in this class typically return {@link Command} objects that can
 * be scheduled or combined with other commands to create
 * comprehensive autonomous or teleoperated sequences.
 */

public class SuperstructureFactory {
    /**
     * Alias for runIntakeAndFeederStagingInShooter with preconfigured constants
     */
    public static Command normalModeIntaking(RobotContainer container) {
        return SuperstructureFactory.runIntakeAndFeederStagingInShooter(container,
                Constants.IntakeConstants.kIntakeDutyCycleIntake,
                Constants.FeederConstants.kFeederRPSIntake).withName("Aim at goal & intake");
    }

    /**
     * Runs another temprary intake job if we think the driver didn't intake enough
     *
     * @param container
     * @return
     */
    public static Command normalModeIntakingEnd(RobotContainer container) {
        return new ConditionalCommand(normalModeIntaking(container).withTimeout(1.5), Commands.none(), () -> {
            // We want to keep intaking if:
            // - no note finished intaking AND one of the following:
            // - Note in intake
            // - Note in pizza box

            return !container.getShooterStage1().hasNote() && (container.getIntake().hadNoteAtIntakeBanner(1.5) ||
                    container.getFeeder().hasNoteAtPizzaBoxBanner());
        });
    }

    public static Command exhaustNote(RobotContainer container) {
        // Command hood at 15 and turret away from intake
        return new ParallelCommandGroup(
                container.getHood().positionSetpointCommand(() -> Units.degreesToRadians(15), () -> 0.0),
                container.getTurret().positionSetpointCommand(() -> Units.degreesToRadians(180.0), () -> 0.0),
                container.getIntake().dutyCycleCommand(() -> Constants.IntakeConstants.kIntakeDutyCycleExhuast),
                container.getAmp().dutyCycleCommand(() -> Constants.AmpConstants.kAmpScoreDutyCycle),
                container.getFeeder().velocitySetpointCommand(
                        () -> -Constants.FeederConstants.kFeederRPSIntake,
                        () -> -Constants.FeederConstants.kFeederRPSIntake),
                container.getShooterStage1()
                        .velocitySetpointCommand(() -> Constants.ShooterConstants.kShooterStage1ExhaustRPS));
    }

    public static Command runAmpIntakeAndFeederStagingInShooter(RobotContainer container,
            double intakeSpeed, double feederSpeed) {
        return new ParallelDeadlineGroup(
                ShooterFactory.intakeUntilStagedInStage1(container),
                AmpFactory.feedFromAmp(container),
                container.getIntake().dutyCycleCommand(() -> intakeSpeed),
                FeederFactory.runBothFeedersTowardsShooter(container, () -> feederSpeed))
                .withName("Run Intake and Feeder");
    }

    /**
     * Run intake, feeder, shooter stage 1 until note is in shooter stage 1.
     * Additionally, stop the intake on a pizza box banner falling edge
     */
    public static Command runIntakeAndFeederStagingInShooter(RobotContainer container,
            double intakeSpeed, double feederSpeed) {
        AtomicBoolean hadNoteInPizzaBox = new AtomicBoolean(false);
        return new SequentialCommandGroup(
                // 1. initialize the falling edge detector for the pizza box banner
                Commands.runOnce(() -> hadNoteInPizzaBox.set(false)),
                // 2. run surfaces
                new ParallelCommandGroup(
                        // 2.a. run shooter stage 1 & feeder until stage 1 banner detects the note
                        new ParallelDeadlineGroup(
                                ShooterFactory.intakeUntilStagedInStage1(container),
                                FeederFactory.runBothFeedersTowardsShooter(container, () -> feederSpeed)),
                        // 2.b. intake until pizza box detects that note left (with a short circuit for
                        // stage 1 seeing note)
                        container.getIntake().dutyCycleCommand(() -> intakeSpeed).until(() -> {
                            boolean hasNoteAtPizza = container.getFeeder().hasNoteAtPizzaBoxBanner();
                            boolean result = !hasNoteAtPizza && hadNoteInPizzaBox.get();
                            hadNoteInPizzaBox.set(hasNoteAtPizza);
                            return result || container.getShooterStage1().hasNote();
                        })),
                // 3. stop surfaces
                new ParallelCommandGroup(
                        container.getIntake().dutyCycleCommand(() -> 0.0),
                        container.getFeeder().dutyCycleCommand(() -> 0, () -> 0)

                ).withTimeout(0.01))
                .withName("Run Intake and Feeder");
    }

    /**
     * For autos & hp mode only only, old stuff that doesn't pause intake
     */
    public static Command runIntakeAndFeederStagingInShooterWithoutIntakeStopping(RobotContainer container,
            double intakeSpeed, double feederSpeed) {
        return new ParallelDeadlineGroup(
                ShooterFactory.intakeUntilStagedInStage1(container),
                container.getIntake().dutyCycleCommand(() -> intakeSpeed),
                FeederFactory.runBothFeedersTowardsShooter(container, () -> feederSpeed))
                .andThen(new ParallelCommandGroup(
                        container.getIntake().dutyCycleCommand(() -> 0.0),
                        container.getFeeder().dutyCycleCommand(() -> 0, () -> 0)

                ).withTimeout(0.02))
                .withName("Run Intake and Feeder");
    }

    public static Command shoot(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        return new ParallelCommandGroup(
                TurretFactory.aimTurretToPose(container, setpointSupplier),
                HoodFactory.aimHoodToPose(container, setpointSupplier),
                ShooterFactory.spinBothStages(container,
                        setpointSupplier),
                FeederFactory.runBothFeedersTowardsShooter(container, () -> Constants.FeederConstants.kFeederRPSIntake),
                VisualizeNoteShot.visualizeNodeShot(container, true))
                .withName("Shoot");
    }

    public static Command feedAndShoot(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        return new ParallelCommandGroup(ShooterFactory.spinBothStages(container,
                setpointSupplier),
                FeederFactory.runBothFeedersTowardsShooter(container, () -> Constants.FeederConstants.kFeederRPSIntake),
                VisualizeNoteShot.visualizeNodeShot(container, true));

    }

    /**
     * Assumes external command is changing shooter speed. Takes ownership of intake
     * For autos, simply use IntakeFactory.runIntake
     */
    public static Command intakeWithBlockingBeforeShooting(RobotContainer container) {
        AtomicBoolean hadNoteInPizzaBox = new AtomicBoolean(false);
        AtomicBoolean hadNoteInShooter = new AtomicBoolean(false);
        return new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    hadNoteInPizzaBox.set(false);
                    hadNoteInShooter.set(false);
                }),
                IntakeFactory.runIntake(container, () -> Constants.IntakeConstants.kIntakeDutyCycleIntake).until(() -> {
                    boolean hasNoteAtPizza = container.getFeeder().hasNoteAtPizzaBoxBanner();
                    boolean result = !hasNoteAtPizza && hadNoteInPizzaBox.get();
                    hadNoteInPizzaBox.set(hasNoteAtPizza);
                    return result;
                }),
                Commands.waitUntil(() -> {
                    boolean hasNoteInShooter = container.getShooterStage1().hasNote();
                    boolean result = !hasNoteInShooter && hadNoteInShooter.get();
                    hadNoteInShooter.set(hasNoteInShooter);
                    return result;
                }).withTimeout(0.3)).repeatedly();
    }

    public static Command ampFeedAndShootAssumeWithOnlyStage1(RobotContainer container,
            Supplier<ShooterSetpoint> setpointSupplier) {
        return new ParallelCommandGroup(ShooterFactory.spinOnlyStageOne(container, setpointSupplier),
                FeederFactory.runBothFeedersTowardsShooter(container, () -> Constants.FeederConstants.kFeederRPSIntake),
                AmpFactory.feedFromAmp(container),
                VisualizeNoteShot.visualizeNodeShot(container, true));
    }

    public static Command feedAndShootAssumeWithOnlyStage1(RobotContainer container,
            Supplier<ShooterSetpoint> setpointSupplier) {
        return new ParallelCommandGroup(ShooterFactory.spinOnlyStageOne(container, setpointSupplier),
                FeederFactory.runBothFeedersTowardsShooter(container, () -> Constants.FeederConstants.kFeederRPSIntake),
                VisualizeNoteShot.visualizeNodeShot(container, true));
    }

    public static Command stopFeedAndShoot(RobotContainer container) {
        return new ParallelCommandGroup(
                container.getShooterStage1().defaultCommand(),
                container.getFeeder().defaultCommand()).withName("Stop Feed And Shoot");
    }

    public static Command spinAndShoot(RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
        return AimFactory.alignUntilOnTarget(container, setpointSupplier)
                .andThen(shoot(container, setpointSupplier))
                .withName("Align then Shoot");
    }

    /**
     * Take a note from the source by raising the elevator, and stows the note in
     * shooter stage 1
     * Exits: when interrupted.
     */
    public static Command intakeFromSource(RobotContainer container) {
        return new ParallelCommandGroup(
                AmpFactory.intakeFromSource(container),
                runIntakeAndFeederStagingInShooter(container,
                        Constants.IntakeConstants.kIntakeDutyCycleIntakeFromSource,
                        Constants.FeederConstants.kFeederRPSIntake))
                .withName("Intake from source");
    }

    /**
     * Runs another temporary intake job if we think the driver didn't intake enough
     */
    public static Command intakeFromSourceEnd(RobotContainer container) {
        return new ConditionalCommand(intakeFromSource(container).withTimeout(1.5), Commands.none(), () -> {
            // We want to keep intaking if:
            // - no note finished intaking AND one of the following:
            // - Note in intake
            // - Note in pizza box
            // - Note in the amp
            return !container.getShooterStage1().hasNote() && (container.getIntake().hadNoteAtIntakeBanner(1.5) ||
                    container.getFeeder().hasNoteAtPizzaBoxBanner() ||
                    container.getAmp().getHasNote());
        });
    }

    /**
     * Exhaust away from shooter for up to 1 second. If Pizza box banner is tripped,
     * keep going.
     * If not, switch to a runDirection for the remaining time
     * <p>
     * *Does not exit*. This command is intended to be raced against another command
     */
    public static Command feedOutOfStage1AmpMode(RobotContainer container) {
        final double timeToWaitForNoteToEnterPizzaBox = 1.0;
        var state = new Object() {
            public boolean sawNoteAtPizzaBoxBanner = false;
            public double startTime = Timer.getFPGATimestamp();
        };
        return new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    state.sawNoteAtPizzaBoxBanner = container.getFeeder().hasNoteAtPizzaBoxBanner();
                    state.startTime = Timer.getFPGATimestamp();
                }),
                FeederFactory.runBothFeedersAwayFromShooter(container, () -> Constants.FeederConstants.kFeederRPSIntake)
                        .until(() -> {
                            // check pizza box
                            state.sawNoteAtPizzaBoxBanner = container.getFeeder().hasNoteAtPizzaBoxBanner();

                            // stop this if:
                            // - timer elapses AND hasn't seen note
                            return !state.sawNoteAtPizzaBoxBanner
                                    && (Timer.getFPGATimestamp() - state.startTime > timeToWaitForNoteToEnterPizzaBox);
                        }),
                FeederFactory.runDirection(container, true,
                        () -> Constants.FeederConstants.kFeederRPSIntake,
                        () -> Constants.FeederConstants.kFeederRPSIntake * 0.75));
    }

    public static Command moveNoteToElevator(RobotContainer container) {
        return new ParallelCommandGroup(
                container.getElevator().motionMagicSetpointCommandBlocking(
                        Constants.ElevatorConstants.kElevatorHomeHeightInches,
                        Constants.ElevatorConstants.kElevatorPositioningToleranceInches),
                ShooterFactory.exhaustStage1(container),
                feedOutOfStage1AmpMode(container),
                IntakeFactory.runIntake(container,
                        () -> Constants.IntakeConstants.kIntakeDutyCycleExhuast));
    }

    public static Command exhaustStage1IntoAmp(RobotContainer container) {
        return new SequentialCommandGroup(
                // exhaust into stage 1 until amp banner tripped
                new ParallelDeadlineGroup(
                        container.getAmp().runUntilPostChopsticBannerGetsNote(
                                () -> Constants.AmpConstants.kAmpExhaustToStageDutyCycle),
                        moveNoteToElevator(container)),
                // halt the feeder and shooter, move note to chopsticks, move elevator up all at
                // once
                new ParallelCommandGroup(
                        AmpFactory.moveElevatorToAmpScoreHeight(container),
                        container.getAmp()
                                .spinRotationsBlocking(Constants.AmpConstants.kAmpChopsticksStageRotations,
                                        Constants.AmpConstants.kAmpChopsticksStageRotationsTolerance),
                        FeederFactory.runBothFeedersTowardsShooter(container, () -> 0.0),
                        container.getShooterStage1().dutyCycleCommand(() -> 0.0)));
    }

    public static Command exhaustStage1IntoElevator(RobotContainer container) {
        return new ParallelDeadlineGroup(
                AmpFactory.stageNoteBeforeLoweredAmpChopsticks(container),
                moveNoteToElevator(container))
                .andThen(container.getAmp().spinRotationsBlocking(-7.0, 0.5));
    }

    public static Command zeroHoodAndPointTurretBackwards(RobotContainer container) {
        final double hoodZeroPositionRadians = Math.toRadians(15.0) + Constants.HoodConstants.kHoodMinPositionRadians;
        final double backwards = Rotation2d.fromRotations(0.5).getRadians();
        return new ParallelCommandGroup(
                new ParallelDeadlineGroup(
                        container.getTurret().waitForPosition(() -> backwards,
                                Constants.TurretConstants.kTurretEpsilon),
                        container.getTurret().positionSetpointCommand(() -> backwards, () -> 0.0)),
                new ParallelDeadlineGroup(
                        container.getHood().waitForPosition(
                                () -> hoodZeroPositionRadians,
                                hoodZeroPositionRadians),
                        container.getHood().positionSetpointCommand(
                                () -> hoodZeroPositionRadians, () -> 0.0)));
    }

    /**
     * Moves a note from stage 1 in the shooter to the lowered elevator as a
     * pre-stage for trapping and amping upon entering the mode.
     * <p>
     * Exits: when the action is complete
     * Uses: Amp rollers, Feeder, intake
     * Assumes: elevator is at home height (with timeout) - and that the hood and
     * turret are in the right place
     */
    public static Command moveNoteFromStage1IntoElevator(RobotContainer container) {
        return new SequentialCommandGroup(
                container.getElevator().positionSetpointUntilOnTargetCommand(
                        () -> Constants.ElevatorConstants.kElevatorHomeHeightInches,
                        () -> Constants.ElevatorConstants.kElevatorPositioningToleranceInches)
                        .withTimeout(0.25),
                // Until the amp has the note in its chopsticks, exhaust the note from the
                // shooter into the amp hood
                exhaustStage1IntoElevator(container))
                .withName("move note to elevator");
    }

    /**
     * Moves a note from the elevator back into the stage 1 by intaking
     * <p>
     * Exits: when the action is complete
     * Uses: Amp rollers, intake, feeder
     * Assumes: elevator is at zero (with timeout)
     */
    public static Command moveNoteFromElevatorBackIntoStage1(RobotContainer container) {
        return new SequentialCommandGroup(
                container.getElevator().positionSetpointUntilOnTargetCommand(
                        () -> Constants.ElevatorConstants.kElevatorHomeHeightInches,
                        () -> Constants.ElevatorConstants.kElevatorPositioningToleranceInches)
                        .withTimeout(0.25),
                new ParallelDeadlineGroup(
                        normalModeIntaking(container).withTimeout(1.5),
                        container.getAmp().dutyCycleCommand(() -> Constants.AmpConstants.kAmpScoreDutyCycle)))
                .withName("move note to from elevator back to stage 1");
    }

    public static Command zeroHoodTurretElevator(RobotContainer container) {
        return new ParallelCommandGroup(
                container.getHood().positionSetpointCommand(() -> 0.0, () -> 0.0),
                container.getTurret().positionSetpointCommand(() -> Math.toDegrees(0.0), () -> 0.0),
                container.getElevator().motionMagicSetpointCommand(() -> 0.0).asProxy());
    }

    public static Command zeroHoodAndTurret(RobotContainer container) {
        return new ParallelCommandGroup(
                container.getHood().positionSetpointCommand(() -> 0.0, () -> 0.0),
                container.getTurret().positionSetpointCommand(() -> Math.toDegrees(0.0), () -> 0.0));
    }

    public static Command stageAmpAndElevatorForTrapThenClimb(RobotContainer container) {
        return new SequentialCommandGroup(
                zeroSubsystemsPreClimb(container),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                stageElevatorForScoringInTrap(container),
                                new ParallelCommandGroup(
                                        container.getClimber().positionSetpointCommand(
                                                () -> Constants.ClimberConstants.kReverseMinPositionRotations),
                                        container.getClimber().waitForClimberPosition(
                                                () -> (Constants.ClimberConstants.kReverseMinPositionRotations
                                                        + Constants.ClimberConstants.kForwardMaxPositionRotations) * 0.5
                                                        + Constants.ClimberConstants.kReverseMinPositionRotations)
                                                .andThen(AmpFactory.moveStagedNoteToTrapPosition(container))
                                                .andThen(container.getElevator().waitForElevatorPosition(
                                                        () -> Constants.ElevatorConstants.kClimbHeightInches))
                                                .andThen(AmpFactory.ampShootPulsing(container))))));
    }

    public static Command getDownFromScore(RobotContainer container) {
        return new SequentialCommandGroup(
                container.getClimber().positionSetpointUntilOnTargetCommand(
                        () -> Constants.ClimberConstants.kForwardMaxPositionRotations,
                        () -> Constants.ElevatorConstants.kElevatorPositioningToleranceInches),
                container.getElevator().motionMagicSetpointCommandBlocking(
                        Constants.ElevatorConstants.kElevatorHomeHeightInches,
                        Constants.ElevatorConstants.kElevatorPositionToleranceRotations));
    }

    public static Command stageElevatorForScoringInTrap(RobotContainer container) {
        return AmpFactory.stageElevatorToScoreTrap(container).withName("Stage Elevator For Trap");
    }

    private static Command zeroSubsystemsPreClimb(RobotContainer container) {
        return new ParallelCommandGroup(
                new ParallelDeadlineGroup(
                        container.getTurret().waitForPosition(
                                () -> Math.toRadians(180.0),
                                Constants.TurretConstants.kTurretEpsilon),
                        container.getTurret().positionSetpointCommand(() -> Math.toRadians(180.0), () -> 0.0)),
                new ParallelDeadlineGroup(
                        container.getHood().waitForPosition(
                                () -> Constants.HoodConstants.kHoodMinPositionRadians,
                                Constants.HoodConstants.kHoodPositionTolerance),
                        container.getHood().positionSetpointCommand(
                                () -> Constants.HoodConstants.kHoodMinPositionRadians, () -> 0.0)));
    }

    public static Command poopDefaultCommand(RobotContainer container, Supplier<ShooterSetpoint> pooper) {
        Command poopDefaultCommand = new ParallelCommandGroup(
                // Align to poop pose
                AimFactory.alignHoodAndTurret(container, pooper),
                // LEDs
                LedFactory.poopModeLEDs(container, () -> AimFactory.onTarget(container, pooper)))
                .withName("Poop Default Command");
        return poopDefaultCommand;
    }

    public static Command poopShoot(RobotContainer container, Supplier<ShooterSetpoint> pooper) {
        Command poopShoot = new ParallelCommandGroup(
                ShooterFactory.spinUpStage2(container, pooper),
                new SequentialCommandGroup(
                        SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(container, pooper),
                        new WaitUntilCommand(() -> AimFactory.onTarget(container, pooper)).andThen(
                                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(container, pooper))))
                .withName("Poop shoot");
        return poopShoot;
    }

    public static Command hpIntakeAndShoot(RobotContainer container, Supplier<ShooterSetpoint> pooper) {
        Command hpIntakeAndShoot = new ParallelCommandGroup(
                ShooterFactory.spinUpStage2(container, pooper),
                new SequentialCommandGroup(
                        SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(container, pooper),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(() -> AimFactory.onTarget(container, pooper))
                                        .andThen(new ParallelCommandGroup(
                                                AmpFactory.intakeFromSource(container),
                                                SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(container,
                                                        pooper))),
                                // for hp intake and shoot don't rate limit notes. this is the difference
                                // between this and poopIntakeAndShoot
                                IntakeFactory.runIntake(container,
                                        () -> Constants.IntakeConstants.kIntakeDutyCycleIntake))))
                .withName("Poop shoot & intake");
        return hpIntakeAndShoot;
    }

    public static Command poopIntakeAndShoot(RobotContainer container, Supplier<ShooterSetpoint> pooper) {
        Command poopIntakeAndShoot = new ParallelCommandGroup(
                ShooterFactory.spinUpStage2(container, pooper),
                new SequentialCommandGroup(
                        SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(container, pooper),
                        new ParallelCommandGroup(
                                new WaitUntilCommand(() -> AimFactory.onTarget(container, pooper)).andThen(
                                        SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(container, pooper)),
                                SuperstructureFactory.intakeWithBlockingBeforeShooting(container))))
                .withName("Poop shoot & intake");
        return poopIntakeAndShoot;
    }

    public static Command stopIntakingandFeedingInitiallyIfNotOnTargetYet(RobotContainer container,
            Supplier<ShooterSetpoint> setpoint) {
        return new ConditionalCommand(Commands.none(),
                new ParallelCommandGroup( // Stop everything before we go if we aren't on target
                        IntakeFactory.runIntake(container, () -> 0.0).withTimeout(0.01),
                        container.getFeeder().dutyCycleCommand(() -> 0.0, () -> 0.0).withTimeout(0.01),
                        container.getShooterStage1().dutyCycleCommand(() -> 0.0).withTimeout(0.01)),
                () -> AimFactory.onTarget(container, setpoint));
    }

}
