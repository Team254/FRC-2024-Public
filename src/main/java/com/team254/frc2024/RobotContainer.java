// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2024;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.team254.frc2024.command_factories.*;
import com.team254.frc2024.subsystems.hood.HoodIOHardware;
import com.team254.frc2024.subsystems.hood.HoodIOSim;
import com.team254.frc2024.subsystems.turret.TurretIOHardware;
import com.team254.frc2024.subsystems.turret.TurretIOSim;
import com.team254.lib.auto.AutoUtil.FirstAction;
import com.team254.lib.auto.AutoUtil.LastAction;
import com.team254.lib.auto.AutoUtil.PriorityMidlineSequence;
import com.team254.lib.auto.AutoUtil.StartingLocation;
import com.team254.lib.ctre.swerve.SwerveRequest;
import com.team254.lib.loops.StatusSignalLoop;
import com.team254.lib.subsystems.SimElevatorIO;
import com.team254.lib.subsystems.SimTalonFXIO;
import com.team254.lib.subsystems.TalonFXIO;
import com.team254.lib.util.MathHelpers;
import com.team254.lib.util.ShooterSetpoint;
import com.team254.lib.util.PoopTargetFactory.NearTarget;

import com.team254.lib.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import com.pathplanner.lib.auto.NamedCommands;
import com.team254.frc2024.AutoModeSelector.DesiredMode;
import com.team254.frc2024.commands.DriveMaintainingHeadingCommand;
import com.team254.frc2024.controlboard.ControlBoard;
import com.team254.frc2024.controlboard.ModalControls;
import com.team254.frc2024.subsystems.amp.AmpSensorIOHardware;
import com.team254.frc2024.subsystems.amp.AmpSubsystem;
import com.team254.frc2024.subsystems.climber.ClimberSubsystem;
import com.team254.frc2024.simulation.SimulatedRobotState;
import com.team254.frc2024.subsystems.drive.*;
import com.team254.frc2024.subsystems.elevator.ElevatorSubsystem;
import com.team254.frc2024.subsystems.feeder.FeederSensorIOHardware;
import com.team254.frc2024.subsystems.feeder.FeederSubsystem;
import com.team254.frc2024.subsystems.hood.HoodSubsystem;
import com.team254.frc2024.subsystems.intake.IntakeSensorIOHardware;
import com.team254.frc2024.subsystems.intake.IntakeSensorIOSim;
import com.team254.frc2024.subsystems.intake.IntakeSubsystem;
import com.team254.frc2024.subsystems.led.LedIOHardware;
import com.team254.frc2024.subsystems.led.LedSubsystem;
import com.team254.frc2024.subsystems.shooter.ShooterStage2Subsystem;
import com.team254.frc2024.subsystems.shooterStage1.ShooterSensorIOHardware;
import com.team254.frc2024.subsystems.shooterStage1.ShooterSensorIOSim;
import com.team254.frc2024.subsystems.shooterStage1.ShooterStage1Subsystem;
import com.team254.frc2024.subsystems.turret.TurretSubsystem;
import com.team254.frc2024.subsystems.vision.VisionFieldPoseEstimate;
import com.team254.frc2024.subsystems.vision.VisionIOHardwareLimelight;
import com.team254.frc2024.subsystems.vision.VisionIOSimPhoton;
import com.team254.frc2024.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private DriveSubsystem buildDriveSystem() {
        if (RobotBase.isSimulation()) {
            return new DriveSubsystem(
                    new DriveIOSim(
                            robotState,
                            simulatedRobotState,
                            Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                    robotState);
        } else {
            return new DriveSubsystem(
                    new DriveIOHardware(
                            robotState,
                            Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                    robotState);
        }
    }

    private TurretSubsystem buildTurret() {
        if (RobotBase.isSimulation()) {
            return new TurretSubsystem(new TurretIOSim(), robotState);
        } else {
            return new TurretSubsystem(new TurretIOHardware(), robotState);
        }
    }

    private ClimberSubsystem buildClimber() {
        if (RobotBase.isSimulation()) {
            return new ClimberSubsystem(Constants.kClimberConfig, new SimTalonFXIO(Constants.kClimberConfig),
                    robotState);
        } else {
            return new ClimberSubsystem(Constants.kClimberConfig, new TalonFXIO(Constants.kClimberConfig), robotState);
        }
    }

    private FeederSubsystem buildFeeder() {
        if (RobotBase.isSimulation()) {
            return new FeederSubsystem(Constants.kFeederLeftConfig, Constants.kFeederRightConfig,
                    new SimTalonFXIO(Constants.kFeederLeftConfig), new SimTalonFXIO(Constants.kFeederRightConfig),
                    new FeederSensorIOHardware());
        } else {
            return new FeederSubsystem(Constants.kFeederLeftConfig, Constants.kFeederRightConfig,
                    new TalonFXIO(Constants.kFeederLeftConfig), new TalonFXIO(Constants.kFeederRightConfig),
                    new FeederSensorIOHardware());
        }
    }

    private ShooterStage1Subsystem buildShooterStage1() {
        if (RobotBase.isSimulation()) {
            return new ShooterStage1Subsystem(Constants.kShooterStage1Config,
                    new SimTalonFXIO(Constants.kShooterStage1Config),
                    simulatedStage1Sensors);
        } else {
            return new ShooterStage1Subsystem(Constants.kShooterStage1Config,
                    new TalonFXIO(Constants.kShooterStage1Config),
                    new ShooterSensorIOHardware(Constants.SensorConstants.kShooterStage1BannerSensorPort));
        }

    }

    private ShooterStage2Subsystem buildShooterStage2() {
        if (RobotBase.isSimulation()) {
            return new ShooterStage2Subsystem(Constants.kShooterStage2BottomConfig,
                    new SimTalonFXIO(Constants.kShooterStage2BottomConfig), Constants.kShooterStage2TopConfig,
                    new SimTalonFXIO(Constants.kShooterStage2TopConfig));
        } else {
            return new ShooterStage2Subsystem(Constants.kShooterStage2BottomConfig,
                    new TalonFXIO(Constants.kShooterStage2BottomConfig), Constants.kShooterStage2TopConfig,
                    new TalonFXIO(Constants.kShooterStage2TopConfig));
        }
    }

    private ElevatorSubsystem buildElevator() {
        if (RobotBase.isSimulation()) {
            return new ElevatorSubsystem(Constants.kElevatorConfig,
                    new SimElevatorIO(Constants.kElevatorConfig, Constants.kElevatorSimConfig), robotState);
        } else {
            return new ElevatorSubsystem(Constants.kElevatorConfig, new TalonFXIO(Constants.kElevatorConfig),
                    robotState);
        }
    }

    private HoodSubsystem buildHood() {
        if (RobotBase.isSimulation()) {
            return new HoodSubsystem(new HoodIOSim(), robotState);
        }
        return new HoodSubsystem(new HoodIOHardware(), robotState);
    }

    private AmpSubsystem buildAmp() {
        if (RobotBase.isSimulation()) {
            return new AmpSubsystem(Constants.kAmpConfig, new SimTalonFXIO(Constants.kAmpConfig),
                    new AmpSensorIOHardware());
        } else {
            return new AmpSubsystem(Constants.kAmpConfig, new TalonFXIO(Constants.kAmpConfig),
                    new AmpSensorIOHardware());
        }
    }

    private IntakeSubsystem buildIntake() {
        if (RobotBase.isSimulation()) {
            return new IntakeSubsystem(Constants.kIntakeConfig, new SimTalonFXIO(Constants.kIntakeConfig),
                    simulatedIntakeSensors, robotState);
        } else {
            return new IntakeSubsystem(Constants.kIntakeConfig, new TalonFXIO(Constants.kIntakeConfig),
                    new IntakeSensorIOHardware(), robotState);
        }
    }

    private LedSubsystem buildLeds() {
        return new LedSubsystem(new LedIOHardware());
    }

    private VisionSubsystem buildVisionSystem() {
        if (RobotBase.isSimulation()) {
            return new VisionSubsystem(new VisionIOSimPhoton(robotState, simulatedRobotState), robotState);
        } else {
            return new VisionSubsystem(new VisionIOHardwareLimelight(robotState), robotState);
        }
    }

    private PowerDistribution powerDistribution = new PowerDistribution();

    // Driver interface
    private final ControlBoard controlBoard = ControlBoard.getInstance();
    private final ModalControls modalControls = ModalControls.getInstance();
    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer = new Consumer<VisionFieldPoseEstimate>() {
        @Override
        public void accept(VisionFieldPoseEstimate estimate) {
            driveSubsystem.addVisionMeasurement(estimate);
        }
    };

    public TurretSubsystem getTurret() {
        return turret;
    }

    public ShooterStage1Subsystem getShooterStage1() {
        return shooterStage1;
    }

    public ShooterStage2Subsystem getShooterStage2() {
        return shooterStage2;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    public HoodSubsystem getHood() {
        return hood;
    }

    public AmpSubsystem getAmp() {
        return amp;
    }

    public FeederSubsystem getFeeder() {
        return feeder;
    }

    public LedSubsystem getLeds() {
        return leds;
    }

    public ElevatorSubsystem getElevator() {
        return elevator;
    }

    public ClimberSubsystem getClimber() {
        return climber;
    }

    public ControlBoard getControlBoard() {
        return controlBoard;
    }

    private final RobotState robotState = new RobotState(visionEstimateConsumer);

    private final StatusSignalLoop statusSignalLoop = new StatusSignalLoop(250.0, "TurretThread");

    private final ShooterSensorIOSim simulatedStage1Sensors = Robot.isSimulation()
            ? new ShooterSensorIOSim(Constants.SensorConstants.kShooterStage1BannerSensorPort)
            : null;
    private final IntakeSensorIOSim simulatedIntakeSensors = Robot.isSimulation() ? new IntakeSensorIOSim() : null;
    private final SimulatedRobotState simulatedRobotState = Robot.isSimulation() ? new SimulatedRobotState(this) : null;
    private final DriveSubsystem driveSubsystem = buildDriveSystem();
    private final TurretSubsystem turret = buildTurret();
    private final ShooterStage1Subsystem shooterStage1 = buildShooterStage1();
    private final ShooterStage2Subsystem shooterStage2 = buildShooterStage2();
    private final FeederSubsystem feeder = buildFeeder();
    private final IntakeSubsystem intake = buildIntake();
    private final HoodSubsystem hood = buildHood();
    private final AmpSubsystem amp = buildAmp();
    private final LedSubsystem leds = buildLeds();

    @SuppressWarnings("unused")
    private final ClimberSubsystem climber = buildClimber();
    private final ElevatorSubsystem elevator = buildElevator();

    private AtomicBoolean climbButtonClimbs = new AtomicBoolean(true);

    @SuppressWarnings("unused")
    private final VisionSubsystem visionSystem = buildVisionSystem();

    private final DriveMaintainingHeadingCommand driveCommand = new DriveMaintainingHeadingCommand(driveSubsystem,
            robotState, controlBoard::getThrottle, controlBoard::getStrafe, controlBoard::getRotation);

    // Auto
    private final AutoModeSelector autoModeSelector;

    public RobotContainer() {
        Supplier<ShooterSetpoint> speakerGoalSupplier = ShooterSetpoint.speakerSetpointSupplier(robotState);
        NamedCommands.registerCommand("AmpFeedAndShoot",
                new WaitUntilCommand(() -> AimFactory.onTarget(this, speakerGoalSupplier))
                        .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(this, speakerGoalSupplier)));
        NamedCommands.registerCommand("FeedAndShoot",
                new ParallelCommandGroup(
                        new WaitUntilCommand(() -> AimFactory.onTarget(this, speakerGoalSupplier))
                                .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(this,
                                        speakerGoalSupplier)),
                        IntakeFactory.runIntake(this, () -> Constants.IntakeConstants.kIntakeDutyCycleIntake)));
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
                SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(this, speakerGoalSupplier),
                new WaitUntilCommand(() -> AimFactory.onTarget(this, speakerGoalSupplier))
                        .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(this,
                                speakerGoalSupplier))));
        NamedCommands.registerCommand("StageInShooter",
                SuperstructureFactory.runIntakeAndFeederStagingInShooterWithoutIntakeStopping(this,
                        Constants.IntakeConstants.kIntakeDutyCycleIntake,
                        Constants.FeederConstants.kFeederRPSIntake));
        NamedCommands.registerCommand("EngagePathCancel", Commands.runOnce(() -> robotState.enablePathCancel()));
        NamedCommands.registerCommand("Override RPS",
                Commands.runOnce(() -> ShooterSetpoint.setOverrideRPS(Constants.ShooterConstants.kPreloadShotRPS)));
        NamedCommands.registerCommand("Clear RPS", Commands.runOnce(() -> ShooterSetpoint.clearOverrideRPS()));

        autoModeSelector = new AutoModeSelector(this);

        configureBindings();
        SmartDashboard.putBoolean("Is Practice Bot", Constants.kIsPracticeBot);
        statusSignalLoop.register(turret);
        statusSignalLoop.register(getAmp());
        statusSignalLoop.register(getShooterStage1());
        statusSignalLoop.register(getIntake());
        statusSignalLoop.start();
    }

    // Button bindings
    SwerveRequest.SwerveDriveBrake xWheels = new SwerveRequest.SwerveDriveBrake();
    Trigger notifyLowOnTime = new Trigger(() -> {
        // when in teleop tether mode the getMatchTime counts up. only do this when
        // counting down
        boolean isTeleopOnField = DriverStation.isTeleopEnabled() && DriverStation.isFMSAttached();
        double matchTime = DriverStation.getMatchTime();
        double ts = Timer.getFPGATimestamp();

        // teleop
        return isTeleopOnField &&
        // 25-35s
                (matchTime >= 15.0 && matchTime <= 35.0) &&
        // for a small amount of time each second
                ((ts - Math.floor(ts)) > 0.700);
    });

    private void configureBindings() {
        modalControls.configureBindings();
        modalControls.registerStateChangeConsumer(new Consumer<ModalControls.Mode>() {
            @Override
            public void accept(ModalControls.Mode mode) {
                turret.updateModeChange();
            }
        });
        driveSubsystem.setDefaultCommand(driveCommand);
        controlBoard.getWantToXWheels()
                .and(() -> Math.abs(controlBoard.getStrafe()) < Constants.kDriveJoystickThreshold)
                .and(() -> Math.abs(controlBoard.getThrottle()) < Constants.kDriveJoystickThreshold)
                .and(() -> Math.abs(controlBoard.getRotation()) < Constants.kJoystickThreshold)
                .whileTrue(driveSubsystem.applyRequest(() -> xWheels));

        controlBoard.resetGyro().debounce(0.5).onTrue(Commands.runOnce(() -> {
            resetHeading();
        }, driveSubsystem));

        controlBoard.exhaust().whileTrue(
                intake.dutyCycleCommand(() -> Constants.IntakeConstants.kIntakeDutyCycleExhuast));
        controlBoard.getExhaustAll().whileTrue(
                SuperstructureFactory.exhaustNote(this));

        // priority over normal modes, yield to climb mode and low on time notification
        controlBoard.getWantTwoNotes()
                .and(modalControls.climbMode().negate())
                .and(notifyLowOnTime.negate())
                .whileTrue(LedFactory.twoNotesLEDs(this));

        // priority over wanting two notes indication, yield to climb mode
        notifyLowOnTime
                .and(modalControls.climbMode().negate())
                .whileTrue(LedFactory.twoNotesLEDs(this).withName("Low on time LED notification"));

        modalControls.noopMode().and(controlBoard.getExhaustAll().negate()).onTrue(
                new ConditionalCommand(
                        SuperstructureFactory.zeroHoodTurretElevator(this),
                        SuperstructureFactory.zeroHoodAndTurret(this),
                        () -> !Util.epsilonEquals(elevator.getCurrentPosition(),
                                Constants.ElevatorConstants.kElevatorHomeHeightInches,
                                Constants.ElevatorConstants.kElevatorPositioningToleranceInches))
                        .withName("noop modes"));

        Supplier<ShooterSetpoint> fenderShotSupplier = () -> new ShooterSetpoint(
                Constants.ShooterConstants.kFenderShotRPS, 0, 0, Constants.HoodConstants.kFenderShotRadians, 0);

        modalControls.noopFenderShot().whileTrue(
                new ParallelCommandGroup(
                        ShooterFactory.spinUpStage2(this, fenderShotSupplier),
                        new SequentialCommandGroup(
                                SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(this,
                                        fenderShotSupplier),
                                new WaitUntilCommand(() -> AimFactory.onTarget(this, fenderShotSupplier)).andThen(
                                        SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(this,
                                                fenderShotSupplier))))
                        .withName("Fender shot"));

        modalControls.noopIntake().whileTrue(SuperstructureFactory.normalModeIntaking(this)
                .andThen(controlBoard.rumble()).withName("Noop mode: intake"));

        modalControls.noopIntake().onFalse(
                new ConditionalCommand(
                        SuperstructureFactory.normalModeIntakingEnd(this),
                        Commands.none(),
                        () -> {
                            return modalControls.noopMode().getAsBoolean()
                                    && !modalControls.noopFenderShot().getAsBoolean()
                                    && !modalControls.noopIntakeAndFenderShot().getAsBoolean();
                        }).withName("Auto-intake stowing (noop mode)"));

        modalControls.noopIntakeAndFenderShot().whileTrue(
                new ParallelCommandGroup(
                        ShooterFactory.spinUpStage2(this, fenderShotSupplier),
                        new SequentialCommandGroup(
                                SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(this,
                                        fenderShotSupplier),
                                new ParallelCommandGroup(
                                        new WaitUntilCommand(() -> AimFactory.onTarget(this, fenderShotSupplier))
                                                .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(this,
                                                        fenderShotSupplier)),
                                        SuperstructureFactory.intakeWithBlockingBeforeShooting(this))))
                        .withName("Fender shot & intake"));

        /////////////////////////////////////////////////////////////////////////////////////////
        // Aim at Goal mode
        Supplier<ShooterSetpoint> speakerGoalSupplier = ShooterSetpoint.speakerSetpointSupplier(robotState);
        modalControls.aimAtGoalMode()
                .and(controlBoard.getWantTwoNotes().negate())
                .and(notifyLowOnTime.negate())
                .whileTrue(new ParallelCommandGroup(
                        // Light up the leds if the goal is pointed at
                        LedFactory.speakerModeLEDs(this, () -> AimFactory.onTarget(this, speakerGoalSupplier)))
                        .withName("Aim at goal mode (LEDs)"));
        modalControls.aimAtGoalMode().and(controlBoard.getExhaustAll().negate()).whileTrue(
                AimFactory.alignHoodAndTurret(this, speakerGoalSupplier));
        modalControls.aimAtGoalIntake().whileTrue(SuperstructureFactory.normalModeIntaking(this)
                .andThen(controlBoard.rumble()).withName("Aim at goal mode: intake"));

        modalControls.aimAtGoalIntake().onFalse(
                new ConditionalCommand(
                        SuperstructureFactory.normalModeIntakingEnd(this),
                        Commands.none(),
                        () -> {
                            return modalControls.aimAtGoalMode().getAsBoolean()
                                    && !modalControls.aimAtGoalShoot().getAsBoolean()
                                    && !modalControls.aimAtGoalIntakeAndShoot().getAsBoolean();
                        }).withName("Auto-intake stowing (speaker mode)"));

        controlBoard.getSpinUpForShoot().and(
                modalControls.aimAtGoalMode()).and(
                        modalControls.aimAtGoalShoot().negate())
                .and(
                        modalControls.aimAtGoalIntakeAndShoot().negate())
                .whileTrue(
                        ShooterFactory.spinUpStage2(this, speakerGoalSupplier).withName("Aim at goal spin up"));

        modalControls.aimAtGoalShoot().whileTrue(
                new ParallelCommandGroup(
                        ShooterFactory.spinUpStage2(this, speakerGoalSupplier),
                        new SequentialCommandGroup(
                                SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(this,
                                        speakerGoalSupplier),
                                new WaitUntilCommand(() -> AimFactory.onTarget(this, speakerGoalSupplier)).andThen(
                                        SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(this,
                                                speakerGoalSupplier))))
                        .withName("Aim at goal & shoot"));

        modalControls.aimAtGoalIntakeAndShoot().whileTrue(
                new ParallelCommandGroup(
                        ShooterFactory.spinUpStage2(this, speakerGoalSupplier),
                        new SequentialCommandGroup(
                                SuperstructureFactory.stopIntakingandFeedingInitiallyIfNotOnTargetYet(this,
                                        speakerGoalSupplier),
                                new ParallelCommandGroup(
                                        new WaitUntilCommand(() -> AimFactory.onTarget(this, speakerGoalSupplier))
                                                .andThen(SuperstructureFactory.feedAndShootAssumeWithOnlyStage1(this,
                                                        speakerGoalSupplier)),
                                        SuperstructureFactory.intakeWithBlockingBeforeShooting(this))))
                        .withName("Aim at goal & shoot & intake"));

        Supplier<ShooterSetpoint> pooper = ShooterSetpoint.poopSetpointSupplier(robotState);

        /////////////////////////////////////////////////////////////////////////////////////////
        // HP Mode
        modalControls.hpMode()
                .and(controlBoard.getWantTwoNotes().negate())
                .and(notifyLowOnTime.negate())
                .whileTrue(new ParallelCommandGroup(
                        // Light up the leds if the goal is pointed at
                        LedFactory.hpModeLEDs(this, () -> AimFactory.onTarget(this, pooper)))
                        .withName("HP Mode (LEDs)"));
        modalControls.hpMode().and(controlBoard.getExhaustAll().negate()).whileTrue(
                AimFactory.alignHoodAndTurret(this, pooper).withName("HP Mode"));
        modalControls.hpMode().and(
                modalControls.hpIntake().negate()).and(
                        modalControls.hpIntakeAndShoot().negate())
                .whileTrue(
                        elevator.motionMagicSetpointCommand(
                                () -> Constants.ElevatorConstants.kElevatorHomeHeightInches));

        controlBoard.getSpinUpForShoot().and(
                modalControls.hpMode()).and(
                        modalControls.hpIntakeAndShoot().negate())
                .and(
                        modalControls.hpShoot().negate())
                .whileTrue(
                        ShooterFactory.spinUpStage2(this, pooper).withName("HP spin up"));

        modalControls.hpIntake().whileTrue(SuperstructureFactory.intakeFromSource(this).withName("HP Intake"));

        modalControls.hpIntake().onFalse(
                new ConditionalCommand(
                        SuperstructureFactory.intakeFromSourceEnd(this),
                        Commands.none(),
                        () -> {
                            return modalControls.hpMode().getAsBoolean() && !modalControls.hpShoot().getAsBoolean()
                                    && !modalControls.hpIntakeAndShoot().getAsBoolean();
                        })
                        .finallyDo(() -> elevator
                                .motionMagicSetpointCommand(() -> Constants.ElevatorConstants.kElevatorHomeHeightInches)
                                .withTimeout(0.01).schedule())
                        .withName("Auto-intake stowing (hp mode)"));

        modalControls.hpShoot().whileTrue(SuperstructureFactory.poopShoot(this, pooper).withName("HP Shoot"));

        modalControls.hpIntakeAndShoot()
                .whileTrue(SuperstructureFactory.hpIntakeAndShoot(this, pooper).withName("HP Mode Intake & Shoot"));

        /////////////////////////////////////////////////////////////////////////////////////////
        // Poop mode

        controlBoard.getWantPoopShallow().onTrue(Commands.runOnce(() -> {
            robotState.setPoopNearTarget(NearTarget.SHALLOW);
            Logger.recordOutput("Poop Near Target", NearTarget.SHALLOW);
        }).ignoringDisable(true));
        controlBoard.getWantPoopShallow().onFalse(Commands.runOnce(() -> {
            robotState.setPoopNearTarget(NearTarget.DEEP_AMP);
            Logger.recordOutput("Poop Near Target", NearTarget.DEEP_AMP);
        }).ignoringDisable(true));

        modalControls.poopShoot().whileTrue(SuperstructureFactory.poopShoot(this, pooper).withName("Poop Shoot"));

        modalControls.poopMode()
                .and(controlBoard.getWantTwoNotes().negate())
                .and(notifyLowOnTime.negate())
                .whileTrue(
                        LedFactory.poopModeLEDs(this, () -> AimFactory.onTarget(this, pooper))
                                .withName("Poop Mode (LEDs)"));
        modalControls.poopMode().and(controlBoard.getExhaustAll().negate()).whileTrue(
                AimFactory.alignHoodAndTurret(this, pooper).withName("Poop Mode"));
        modalControls.poopIntake()
                .whileTrue(SuperstructureFactory.normalModeIntaking(this).withName("Poop Mode Intake"));

        controlBoard.getSpinUpForShoot().and(
                modalControls.poopMode()).and(
                        modalControls.poopIntakeAndShoot().negate())
                .and(
                        modalControls.poopShoot().negate())
                .whileTrue(
                        ShooterFactory.spinUpStage2(this, pooper).withName("Poop spin up"));

        modalControls.poopIntake().onFalse(
                new ConditionalCommand(
                        SuperstructureFactory.normalModeIntakingEnd(this),
                        Commands.none(),
                        () -> {
                            return modalControls.poopMode().getAsBoolean() && !modalControls.poopShoot().getAsBoolean()
                                    && !modalControls.poopIntakeAndShoot().getAsBoolean();
                        }).withName("Auto-intake stowing (poop mode)"));

        modalControls.poopIntakeAndShoot()
                .whileTrue(SuperstructureFactory.poopIntakeAndShoot(this, pooper).withName("Poop Mode Intake & Shoot"));

        /////////////////////////////////////////////////////////////////////////////////////////
        // Climb mode + Amp Mode

        Trigger stageNotesInElevator = modalControls.ampMode().or(modalControls.climbMode());
        // enter amp or climb -> note into elevator
        stageNotesInElevator.onTrue(SuperstructureFactory.moveNoteFromStage1IntoElevator(this).withTimeout(2.0));
        // leave amp or climb -> note into shooter
        stageNotesInElevator.onFalse(SuperstructureFactory.moveNoteFromElevatorBackIntoStage1(this).withTimeout(2.0));

        /////////////////////////////////////////////////////////////////////////////////////////
        // Amp mode

        Supplier<ShooterSetpoint> pointAtIntakeSupplier = () -> new ShooterSetpoint(0.0,
                Rotation2d.fromDegrees(180.0).getRadians(), 0, Rotation2d.fromDegrees(15.0).getRadians(), 0);

        modalControls.ampMode()
                .and(controlBoard.getWantTwoNotes().negate())
                .and(notifyLowOnTime.negate())
                .whileTrue(new ParallelCommandGroup(
                        // Point at the intake
                        // Light up the leds if the goal is pointed at
                        LedFactory.ampModeLEDs(this)).withName("Amp scoring mode (LEDs)"));
        modalControls.ampMode().and(controlBoard.getExhaustAll().negate()).whileTrue(
                AimFactory.alignHoodAndTurret(this, pointAtIntakeSupplier).withName("Amp scoring mode"));
        modalControls.ampIntake().whileTrue(SuperstructureFactory.normalModeIntaking(this)
                .andThen(controlBoard.rumble()).withName("Amp mode: intake"));

        BooleanSupplier ampIntakeEndPredicate = () -> modalControls.ampMode().getAsBoolean()
                && !modalControls.ampShoot().getAsBoolean();
        modalControls.ampIntake().onFalse(
                new ConditionalCommand(
                        SuperstructureFactory.normalModeIntakingEnd(this).asProxy()
                                .andThen(
                                        new ConditionalCommand(
                                                SuperstructureFactory.moveNoteFromStage1IntoElevator(this)
                                                        .withTimeout(1.5).asProxy(),
                                                Commands.none(),
                                                ampIntakeEndPredicate)),
                        Commands.none(),
                        ampIntakeEndPredicate).withName("Auto-intake stowing (amp mode)"));

        // Held
        modalControls.ampShoot().whileTrue(
                SuperstructureFactory.exhaustStage1IntoAmp(this)
                        // get ready to score
                        .andThen(new ParallelCommandGroup(
                                AmpFactory.moveElevatorToAmpScoreHeight(this),
                                FeederFactory.runBothFeedersTowardsShooter(this, () -> 0.0),
                                shooterStage1.dutyCycleCommand(() -> 0.0)))
                        .withName("Amp: move note to chopsticks and raise"));

        // Released
        modalControls.ampShoot().onFalse(new SequentialCommandGroup(
                AmpFactory.moveElevatorToAmpScoreHeight(this),
                amp.dutyCycleCommand(() -> Constants.AmpConstants.kAmpScoreDutyCycle).withTimeout(0.5),
                AmpFactory.moveElevatorToHome(this)).withName("Amp raise, score, lower"));

        /////////////////////////////////////////////////////////////////////////////////////////
        // Climb mode
        modalControls.climbMode().onTrue(new ParallelCommandGroup(
                LedFactory.climbModeLEDs(this),
                new SequentialCommandGroup(
                        SuperstructureFactory.zeroHoodAndPointTurretBackwards(this),
                        SuperstructureFactory.moveNoteFromStage1IntoElevator(this).withTimeout(2.0),
                        feeder.dutyCycleCommand(() -> 0, () -> 0).withTimeout(0.01),
                        shooterStage1.dutyCycleCommand(() -> 0).withTimeout(0.01)))
                .withName("Climb Mode"));
        modalControls.climbMode()
                .onTrue(climber
                        .positionSetpointUntilOnTargetCommand(() -> Constants.ClimberConstants.kStageHooksRotations,
                                () -> Constants.ClimberConstants.kPositionToleranceRotations)
                        .withName("Stage Hooks For Climb"));
        modalControls.climbMode().onFalse(climber
                .positionSetpointUntilOnTargetCommand(() -> 0.0,
                        () -> Constants.ClimberConstants.kPositionToleranceRotations));
        modalControls.raiseClimbUp().onTrue(
                climber.positionSetpointUntilOnTargetCommand(() -> Constants.ClimberConstants.kHooksUpPositionRotations,
                        () -> Constants.ClimberConstants.kPositionToleranceRotations)
                        .withName("Climber Arms Moving Up"));
        modalControls.climbAndStageForTrap().and(() -> climbButtonClimbs.get()).onTrue(
                Commands.runOnce(() -> climbButtonClimbs.set(false))
                        .andThen(SuperstructureFactory.stageAmpAndElevatorForTrapThenClimb(this))
                        .withName("Staging amp for scoring and climb arms down"));
        modalControls.moveClimbArmsDown().onTrue(
                climber.positionSetpointUntilOnTargetCommand(() -> 0.0,
                        () -> Constants.ClimberConstants.kPositionToleranceRotations)
                        .withName("Move climber hooks down"));
        modalControls.moveClimbDutyCycle().whileTrue(
                climber.jogAndThenZero().withName("Jog and then Zero"));
    }

    public void resetHeading() {
        driveSubsystem.resetOdometry(
                new Pose2d(
                        new Translation2d(
                                robotState.getLatestFieldToRobot().getValue().getX(),
                                robotState.getLatestFieldToRobot().getValue().getY()),
                        robotState.isRedAlliance() ? MathHelpers.kRotation2dPi : MathHelpers.kRotation2dZero));
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public ShooterSensorIOSim getSimulatedStage1Sensors() {
        return simulatedStage1Sensors;
    }

    public IntakeSensorIOSim getSimulatedIntakeSensors() {
        return simulatedIntakeSensors;
    }

    public ModalControls getModalControls() {
        return modalControls;
    }

    public SimulatedRobotState getSimulatedRobotState() {
        return simulatedRobotState;
    }

    public LoggedDashboardChooser<DesiredMode> getModeChooser() {
        return autoModeSelector.getModeChooser();
    }

    public LoggedDashboardString getBlockedMidlineNotes() {
        return autoModeSelector.getBlockedMidlineNotes();
    }

    public LoggedDashboardChooser<PriorityMidlineSequence> getPriorityMidlineSequenceChooser() {
        return autoModeSelector.getPriorityMidlineSequenceChooser();
    }

    public LoggedDashboardChooser<StartingLocation> getStartingLocationChooser() {
        return autoModeSelector.getStartingLocationChooser();
    }

    public LoggedDashboardChooser<FirstAction> getFirstActionChooser() {
        return autoModeSelector.getFirstActionChooser();
    }

    public LoggedDashboardChooser<LastAction> getLastActionChooser() {
        return autoModeSelector.getLastActionChooser();
    }

    public boolean isValidAutoCommand() {
        return autoModeSelector.isValidCommand();
    }

    public void setAutoDefaultCommands() {
        feeder.setDefaultCommand(feeder.dutyCycleCommand(() -> 0.0, () -> 0.0));
        var speakerSetpointSupplier = ShooterSetpoint.speakerSetpointSupplier(robotState);
        turret.setDefaultCommand(TurretFactory.aimTurretToPose(this, speakerSetpointSupplier));
        hood.setDefaultCommand(HoodFactory.aimHoodToPose(this, speakerSetpointSupplier));
        shooterStage2.setDefaultCommand(ShooterFactory.spinUpStage2(this, speakerSetpointSupplier));
    }

    public void setTeleopDefaultCommands() {
        turret.setTeleopDefaultCommand();
        hood.setTeleopDefaultCommand();
        shooterStage2.setTeleopDefaultCommand();
        feeder.setTeleopDefaultCommand();
        intake.setTeleopDefaultCommand();
        ShooterSetpoint.clearOverrideRPS();
    }

    public boolean odometryCloseToPose(Pose2d pose) {
        Pose2d fieldToRobot = robotState.getLatestFieldToRobot().getValue();
        double distance = fieldToRobot.getTranslation().getDistance(pose.getTranslation());
        SmartDashboard.putNumber("Distance From Start Pose", distance);
        double rotation = Math.abs(fieldToRobot.getRotation().rotateBy(pose.getRotation().unaryMinus()).getDegrees());
        SmartDashboard.putNumber("Rotation From Start Pose", rotation);
        if (distance < 0.25 && rotation < 8.0) {
            return true;
        }
        return false;
    }

    public Command getAutonomousCommand() {
        return autoModeSelector.getAutonomousCommand();
    }

    public Command getDisabledCommand() {
        return new ParallelCommandGroup(
                LedFactory.batteryLEDs(this, () -> powerDistribution.getVoltage())
        // Add any other subsystem coast commands here
        ).withName("Disabled command group");
    }
}
