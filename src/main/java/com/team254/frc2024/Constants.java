package com.team254.frc2024;

import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.frc2024.subsystems.drive.CompTunerConstants;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import com.team254.lib.subsystems.SimElevatorIO;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.List;
import java.util.Map;

import com.team254.frc2024.subsystems.drive.CommandSwerveDrivetrain;
import com.team254.frc2024.subsystems.drive.PracTunerConstants;
import com.team254.lib.auto.AutoUtil.MidlineNote;
import com.team254.lib.auto.AutoUtil.PriorityMidlineSequence;
import com.team254.lib.drivers.CANDeviceId;

/**
 * Class to store all constants for robot code.
 */
public class Constants {
    public static final String kCanBusCanivore = "canivore";
    public static boolean kIsReplay = false;
    public static final String kPracticeBotMacAddress = "00:80:2F:33:D1:4B";
    public static boolean kIsPracticeBot = hasMacAddress(kPracticeBotMacAddress);

    public static final double kSteerJoystickDeadband = 0.05;

    public static final ClosedLoopRampsConfigs makeDefaultClosedLoopRampConfig() {
        return new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(0.02)
                .withTorqueClosedLoopRampPeriod(0.02)
                .withVoltageClosedLoopRampPeriod(0.02);
    }

    public static final OpenLoopRampsConfigs makeDefaultOpenLoopRampConfig() {
        return new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.02)
                .withTorqueOpenLoopRampPeriod(0.02)
                .withVoltageOpenLoopRampPeriod(0.02);
    }

    public static final class DriveConstants {
        public static final double kDriveMaxSpeed = 4.43676260556;
        public static final double kDriveMaxAngularRate = Units.degreesToRadians(360 * 1.15);
        public static final double kHeadingControllerP = 3.5;
        public static final double kHeadingControllerI = 0;
        public static final double kHeadingControllerD = 0;
        public static final CommandSwerveDrivetrain kDrivetrain = kIsPracticeBot
                ? PracTunerConstants.DriveTrain
                : CompTunerConstants.DriveTrain;
    }

    public static final class SensorConstants {
        public static final int kAmpPostChopstickSensorPort = 4;
        public static final int kShooterStage1BannerSensorPort = 3;
        public static final int kAmpBannerSensorPort = 2;
        public static final int kFeederBannerSensorPort = 1;
        public static final int kIntakeBannerSensorPort = 0;
        public static final double kShooterDebounceTime = 0.01;
        public static final double kAmpDebounceTime = 0.01;
        public static final double kFeederDebounceTime = 0.01;
        public static final double kIntakeDebounceTime = 0.01;
    }

    public static final class ClimberConstants {
        public static final CANDeviceId kClimberTalonCanID = new CANDeviceId(24, kCanBusCanivore);
        public static final double kClimberP = kIsPracticeBot ? 1.0 : 1.0;
        public static final double kForwardMaxPositionRotations = kIsPracticeBot ? 119.0 : 132.0;
        public static final double kHooksUpPositionRotations = kForwardMaxPositionRotations * 0.9;
        public static final double kStageHooksRotations = kForwardMaxPositionRotations * 0.4;
        public static final double kClimbClimbedPositionToleranceRotations = kForwardMaxPositionRotations * 0.1;
        public static final double kPositionToleranceRotations = 2.0;
        public static final double kClimberGearRatio = 1.0 / (10.0);
        public static double kReverseMinPositionRotations = 0.0;
    }

    public static final ServoMotorSubsystemConfig kClimberConfig = new ServoMotorSubsystemConfig();
    static {
        kClimberConfig.name = "Climber";
        kClimberConfig.talonCANID = new CANDeviceId(24, kCanBusCanivore);
        kClimberConfig.kMaxPositionUnits = ClimberConstants.kForwardMaxPositionRotations;
        kClimberConfig.kMinPositionUnits = 0.0;
        kClimberConfig.fxConfig.Slot0.kP = 1.0 * 12.0;
        kClimberConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kClimberConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.kForwardMaxPositionRotations
                - Constants.ClimberConstants.kPositionToleranceRotations;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        kClimberConfig.fxConfig.Audio.BeepOnBoot = false;
        kClimberConfig.fxConfig.Audio.BeepOnConfig = false;
        kClimberConfig.unitToRotorRatio = 1.0;

        kClimberConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 150.0;
        kClimberConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kClimberConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kClimberConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
        kClimberConfig.momentOfInertia = 0.05;
    }

    public static final class TurretConstants {
        public static final double kTurretGearRatio = (14. / 52.) * (10. / 125.);
        public static final CANDeviceId kTurretTalonCanID = new CANDeviceId(21, kCanBusCanivore);
        public static final CANDeviceId kTurret1To1CANCoder = new CANDeviceId(13, kCanBusCanivore);
        public static final CANDeviceId kTurret3To1CANCoder = new CANDeviceId(14, kCanBusCanivore);
        public static final double k3To1TurretCancoderOffset = kIsPracticeBot ? -0.254150 : -0.057617;
        public static final double k1To1TurretCancoderOffset = kIsPracticeBot ? 0.270996 : 0.482178;
        public static final double kTurretMinPositionRadians = -2. * Math.PI;
        public static final double kTurretMaxPositionRadians = 2. * Math.PI;

        public static final double kTurretEpsilon = Units.degreesToRadians(2.0);
        public static final double kTurretShootingEpsilon = Units.degreesToRadians(5.0);
    }

    public static final class ElevatorConstants {
        public static final double ElevatorMinPositionRotations = 0.0;
        public static final double ElevatorMaxPositionRotations = 15.356933;
        public static final double ElevatorMaxHeightInches = 16.5;
        public static final double kElevatorGearRatio = (11.0 / 36.0) * (18. / 15.);
        public static final double kElevatorPositionToleranceRotations = 0.1;
        public static final double kAmpScoringHeightInches = 16.0;
        public static final double kElevatorHomeHeightInches = 0.0;
        public static final double kIntakeFromSourceHeightInches = 14.5;
        public static final double kElevatorPositioningToleranceInches = 0.5;
        public static final double kClimbHeightInches = 16.0;
        public static final double kSpoolDiameter = Units.inchesToMeters(0.940);
    }

    public static final ServoMotorSubsystemConfig kElevatorConfig = new ServoMotorSubsystemConfig();
    public static final SimElevatorIO.SimElevatorConfig kElevatorSimConfig = new SimElevatorIO.SimElevatorConfig();
    static {
        kElevatorConfig.name = "Elevator";
        kElevatorConfig.talonCANID = new CANDeviceId(23, kCanBusCanivore);
        kElevatorConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kElevatorConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (ElevatorConstants.ElevatorMaxHeightInches
                - 0.25) / ElevatorConstants.ElevatorMaxHeightInches * ElevatorConstants.ElevatorMaxPositionRotations;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        kElevatorConfig.fxConfig.Slot0.kG = 0.26;
        kElevatorConfig.fxConfig.Slot0.kS = 0.18;
        kElevatorConfig.fxConfig.Slot0.kV = 0.135;
        kElevatorConfig.fxConfig.Slot0.kA = 0.0001 * 12.0;
        kElevatorConfig.fxConfig.Slot0.kP = 3.0;

        kElevatorConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 800;
        kElevatorConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        kElevatorConfig.unitToRotorRatio = ElevatorConstants.ElevatorMaxHeightInches
                / (ElevatorConstants.ElevatorMaxPositionRotations - ElevatorConstants.ElevatorMinPositionRotations);
        kElevatorConfig.kMinPositionUnits = 0.0;
        kElevatorConfig.kMaxPositionUnits = ElevatorConstants.ElevatorMaxHeightInches;

        kElevatorConfig.fxConfig.Audio.BeepOnBoot = false;
        kElevatorConfig.fxConfig.Audio.BeepOnConfig = false;

        kElevatorConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        kElevatorConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kElevatorConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kElevatorConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();

        kElevatorSimConfig.gearing = ElevatorConstants.kElevatorGearRatio;
        kElevatorSimConfig.carriageMass = Units.lbsToKilograms(7.98);
        kElevatorSimConfig.drumRadius = ElevatorConstants.kSpoolDiameter;
    }

    // Intake Constants
    public static final class IntakeConstants {
        public static final double kIntakeDutyCycleIntake = 1.;
        public static final double kIntakeDutyCycleIntakeFromSource = 1.0;
        public static final double kIntakeDutyCycleExhuast = -1.0;
    }

    public static final ServoMotorSubsystemConfig kIntakeConfig = new ServoMotorSubsystemConfig();
    static {
        kIntakeConfig.name = "Intake";
        kIntakeConfig.talonCANID = new CANDeviceId(22, kCanBusCanivore);
        kIntakeConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kIntakeConfig.fxConfig.Audio.BeepOnBoot = false;
        kIntakeConfig.fxConfig.Audio.BeepOnConfig = false;

        kIntakeConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kIntakeConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 150.0;
        kIntakeConfig.fxConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        kIntakeConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        kIntakeConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kIntakeConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kIntakeConfig.fxConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;
        kIntakeConfig.fxConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0.05;
        kIntakeConfig.fxConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
    }

    // Feeder Constants
    public static final class FeederConstants {
        public static final double kFeederRPSIntake = 70.0;
        public static final boolean kRunClockwise = true;
    }

    // Left feeder is the left feeder looking at turret @ 0 radians from intake side
    // of robot.
    public static final ServoMotorSubsystemConfig kFeederLeftConfig = new ServoMotorSubsystemConfig();
    public static final ServoMotorSubsystemConfig kFeederRightConfig = new ServoMotorSubsystemConfig();
    static {
        kFeederLeftConfig.name = "LeftFeeder";
        kFeederLeftConfig.talonCANID = new CANDeviceId(17, kCanBusCanivore);

        kFeederRightConfig.name = "RightFeeder";
        kFeederRightConfig.talonCANID = new CANDeviceId(18, kCanBusCanivore);
        kFeederRightConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kFeederRightConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kFeederLeftConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kFeederLeftConfig.fxConfig.Slot0.kS = kFeederRightConfig.fxConfig.Slot0.kS = 0.18;
        kFeederLeftConfig.fxConfig.Slot0.kP = kFeederRightConfig.fxConfig.Slot0.kP = 0.4;
        kFeederLeftConfig.fxConfig.Slot0.kV = kFeederRightConfig.fxConfig.Slot0.kV = 0.126;

        kFeederLeftConfig.unitToRotorRatio = kFeederRightConfig.unitToRotorRatio = 15.0 / 18.0;

        kFeederRightConfig.fxConfig.Audio.BeepOnBoot = false;
        kFeederRightConfig.fxConfig.Audio.BeepOnConfig = false;
        kFeederLeftConfig.fxConfig.Audio.BeepOnBoot = false;
        kFeederLeftConfig.fxConfig.Audio.BeepOnConfig = false;

        kFeederRightConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        kFeederRightConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kFeederRightConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kFeederRightConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();

        kFeederLeftConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        kFeederLeftConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kFeederLeftConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kFeederLeftConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
    }

    public static final class HoodConstants {
        public static final CANDeviceId kHoodTalonCanID = new CANDeviceId(19, kCanBusCanivore);
        public static final double kHoodGearRatio = (14.0 / 48.0) * (15.0 / 36.0) * (10.0 / 160.0);
        public static final double kHoodRotorMaxPosition = 15.368164;
        public static final double kHoodRotorMinPosition = 0.0;
        public static final double kHoodPositionTolerance = 0.1;
        public static final double kHoodMinPositionRadians = 0.0;
        public static final double kHoodMaxPositionRadians = Units
                .rotationsToRadians(kHoodRotorMaxPosition * kHoodGearRatio);
        public static final double kHoodZeroedAngleDegrees = 51.7;
        public static final double kHoodEpsilon = Units.degreesToRadians(1.0);
        public static final double kHoodShootingEpsilon = Units.degreesToRadians(5.0);

        public static final double kFenderShotRadians = Units.degreesToRadians(0);
    }

    public static final ServoMotorSubsystemConfig kShooterStage1Config = new ServoMotorSubsystemConfig();
    public static final ServoMotorSubsystemConfig kShooterStage2TopConfig = new ServoMotorSubsystemConfig();
    public static final ServoMotorSubsystemConfig kShooterStage2BottomConfig = new ServoMotorSubsystemConfig();
    static {
        kShooterStage1Config.name = "Shooter Stage 1";
        kShooterStage2TopConfig.name = "Shooter Stage 2 (Top)";
        kShooterStage2BottomConfig.name = "Shooter Stage 2 (Bottom)";
        kShooterStage1Config.talonCANID = new CANDeviceId(16, kCanBusCanivore);
        kShooterStage1Config.fxConfig.Slot0.kP = 0.25;
        kShooterStage1Config.fxConfig.Slot0.kV = 0.123;
        kShooterStage1Config.unitToRotorRatio = (12.0 / 30.0) * (15.0 / 30.0);

        kShooterStage2TopConfig.talonCANID = new CANDeviceId(15, kCanBusCanivore);
        kShooterStage2TopConfig.fxConfig.Slot0.kS = 0.18;
        kShooterStage2TopConfig.fxConfig.Slot0.kP = 0.4;
        kShooterStage2TopConfig.fxConfig.Slot0.kV = 0.13;
        kShooterStage2TopConfig.unitToRotorRatio = 32.0 / 18.0;
        kShooterStage2TopConfig.momentOfInertia = 0.00050422 + 0.0009987;

        kShooterStage2BottomConfig.talonCANID = new CANDeviceId(26, kCanBusCanivore);
        kShooterStage2BottomConfig.fxConfig.Slot0.kS = 0.18;
        kShooterStage2BottomConfig.fxConfig.Slot0.kP = 0.4;
        kShooterStage2BottomConfig.fxConfig.Slot0.kV = 0.128;
        kShooterStage2BottomConfig.unitToRotorRatio = 32.0 / 18.0;
        kShooterStage2BottomConfig.momentOfInertia = 0.00050422 + 0.0009987;

        kShooterStage1Config.fxConfig.Audio.BeepOnBoot = false;
        kShooterStage1Config.fxConfig.Audio.BeepOnConfig = false;
        kShooterStage2TopConfig.fxConfig.Audio.BeepOnBoot = false;
        kShooterStage2TopConfig.fxConfig.Audio.BeepOnConfig = false;
        kShooterStage2BottomConfig.fxConfig.Audio.BeepOnBoot = false;
        kShooterStage2BottomConfig.fxConfig.Audio.BeepOnConfig = false;

        kShooterStage1Config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kShooterStage2TopConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kShooterStage2BottomConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kShooterStage1Config.fxConfig.CurrentLimits.StatorCurrentLimit = 160.0;
        kShooterStage1Config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kShooterStage1Config.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kShooterStage1Config.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
        kShooterStage1Config.momentOfInertia = 0.0050422;

        kShooterStage2TopConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        kShooterStage2TopConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kShooterStage2TopConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kShooterStage2TopConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();

        kShooterStage2BottomConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        kShooterStage2BottomConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kShooterStage2BottomConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kShooterStage2BottomConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();

    }

    public static class AmpConstants {
        public static final double kAmpIntakeFromSourceDutyCycle = -1.0;
        public static final double kAmpExhaustToStageDutyCycle = 0.85;
        public static final double kAmpSlowlyStageDutyCycle = 0.1;

        public static final double kAmpScoreDutyCycle = -1.0;

        // Number of rotations to turn after beam break is tripped (rising edge) to stow
        // the note prior to scoring
        public static final double kAmpChopsticksStageRotations = 6.0;
        public static final double kTrapChopsticksStageRotations = 10.0;
        public static final double kAmpChopsticksGoBackRotations = -1.75;
        public static final double kAmpChopsticksGoBackRotationsTolerance = 0.25;
        public static final double kAmpChopsticksStageRotationsTolerance = 0.1;
    }

    public static final ServoMotorSubsystemConfig kAmpConfig = new ServoMotorSubsystemConfig();
    static {
        kAmpConfig.name = "Amp";
        kAmpConfig.talonCANID = new CANDeviceId(25, kCanBusCanivore);
        kAmpConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kAmpConfig.fxConfig.Audio.BeepOnBoot = false;
        kAmpConfig.fxConfig.Audio.BeepOnConfig = false;

        kAmpConfig.fxConfig.Slot0.kS = 0.015 * 12.0;
        kAmpConfig.fxConfig.Slot0.kP = 0.3 * 12.0;
        kAmpConfig.fxConfig.Slot0.kV = 0.00925 * 12.0;
        kAmpConfig.fxConfig.Slot0.kA = 0.0001 * 12.0;
        kAmpConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 500.0;
        kAmpConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0;

        kAmpConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kAmpConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        kAmpConfig.fxConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        kAmpConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        kAmpConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kAmpConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kAmpConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
    }

    public static final class ShooterConstants {
        public static final Rotation2d kTurretToShotCorrection = new Rotation2d(Units.degreesToRadians(1.5));

        public static final double kPoopMaxApexHeight = Units.inchesToMeters(160.0);

        public static final double kStage2ShooterWheelDiameter = Units.inchesToMeters(3.0); // in
        public static final double kStage1ShooterWheelDiameter = Units.inchesToMeters(2.0); // in

        public static final double kRingLaunchVelMetersPerSecPerRotPerSec = 0.141;
        public static final double kRingLaunchLiftCoeff = 0.013; // Multiply by v^2 to get lift accel
        public static final double kShooterStage2RPSShortRange = 120.0; // rot/s
        public static final double kShooterStage2MaxShortRangeDistance = 2.0;
        public static final double kShooterStage2MinLongRangeDistance = 3.0;
        public static final double kShooterStage2RPSLongRange = 120.0; // rot/s
        public static final double kShooterStage2RPSCap = 130.0; // rot/s
        public static final double kShooterStage1RPS = 70.0; // rot/s
        public static final double kShooterStage2Epsilon = 3.0;
        public static final double kShooterSpinupStage1RPS = 0.0;

        public static final double kShooterStage1IntakeRPS = 4.0;
        public static final double kShooterStage1ExhaustRPS = -10.0;

        public static final double kFenderShotRPS = 100.0;
        public static final double kPreloadShotRPS = 90.0;

        public static final double kBottomRollerSpeedupFactor = 1.0; // multiplied to all setpoints to determine how
                                                                     // much
                                                                     // extra power to give the bottom roller. >1.0 =
                                                                     // faster
                                                                     // bottom roller
        public static final double kTopRollerSpeedupFactor = 1.0;
    }

    public static final double kJoystickThreshold = 0.1;
    public static final int kDriveGamepadPort = 0;

    // Controls
    public static final boolean kForceDriveGamepad = true;
    public static final int kGamepadAdditionalControllerPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kDriveJoystickThreshold = 0.03;

    // Limelight constants

    // TURRET LIMELIGHT
    // Pitch angle: How many radians the camera is pitched up around Y axis. 0 is
    // looking straight ahead, +is nodding up.
    public static final double kCameraPitchDegrees = kIsPracticeBot ? 28.0 : 27.5;
    public static final double kCameraPitchRads = Units.degreesToRadians(kCameraPitchDegrees);
    public static final double kCameraHeightOffGroundMeters = kIsPracticeBot ? Units.inchesToMeters(11.181)
            : Units.inchesToMeters(11.181);
    public static final double kImageCaptureLatency = 11.0; // milliseconds
    public static final double kLimelightTransmissionTimeLatency = 0.0; // milliseconds
    public static final String kLimelightTableName = "limelight-turret";
    // Distance from turret center to camera lens in X axis (straight into lens)
    public static final double kTurretToCameraX = kIsPracticeBot ? Units.inchesToMeters(5.834)
            : Units.inchesToMeters(5.834);
    // Distance from turret center to camera lens in Y
    public static final double kTurretToCameraY = 0;

    // ELEVATOR LIMELIGHT
    public static final String kLimelightBTableName = "limelight-eleva";
    public static final double kCameraBPitchDegrees = kIsPracticeBot ? 15.0 : 16.0;
    public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
    public static final double kCameraBRollDegrees = kIsPracticeBot ? 0.0 : 0.0;
    public static final double kCameraBRollRads = Units.degreesToRadians(kCameraBRollDegrees);
    public static final double kCameraBHeightOffGroundMeters = kIsPracticeBot ? Units.inchesToMeters(19.477)
            : Units.inchesToMeters(19.477); // verify for practice
    // Distance from turret center to camera lens in X axis (straight into lens)
    public static final double kTurretToCameraBX = kIsPracticeBot ? Units.inchesToMeters(14.882)
            : Units.inchesToMeters(14.882); // verify for practice
    // Distance from turret center to camera lens in Y
    public static final double kTurretToCameraBY = 0;

    public static final double kTurretToRobotCenterX = Units.inchesToMeters(2.3115);
    public static final double kTurretToRobotCenterY = 0;
    public static final Transform2d kTurretToRobotCenter = new Transform2d(
            new Translation2d(Constants.kTurretToRobotCenterX, Constants.kTurretToRobotCenterY),
            new Rotation2d());
    public static final Rotation2d kCameraYawOffset = new Rotation2d(0);

    // April Tag Layout
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth(); // distance between field walls,
                                                                                    // 8.211m
    public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength(); // distance between driver station
                                                                                      // walls, 16.541m
    public static final Map<MidlineNote, Translation2d> kMidlineNoteTranslations = Map.ofEntries(
            Map.entry(MidlineNote.A, new Translation2d(kFieldLengthMeters / 2.0 /* 8.2705 */, 7.47)),
            Map.entry(MidlineNote.B, new Translation2d(kFieldLengthMeters / 2.0, 5.79)),
            Map.entry(MidlineNote.C, new Translation2d(kFieldLengthMeters / 2.0, kFieldWidthMeters / 2.0)),
            Map.entry(MidlineNote.D, new Translation2d(kFieldLengthMeters / 2.0, kFieldWidthMeters - 5.79 /* 2.421 */)),
            Map.entry(MidlineNote.E,
                    new Translation2d(kFieldLengthMeters / 2.0, kFieldWidthMeters - 7.47 /* 0.741 */)));

    public static final Pose2d kBlueAmpPose = new Pose2d(1.820, 7.680, Rotation2d.fromDegrees(90.0));
    public static final Pose2d kRedAmpPose = new Pose2d(kFieldWidthMeters - kBlueAmpPose.getX(), kBlueAmpPose.getY(),
            Rotation2d.fromDegrees(180 - kBlueAmpPose.getRotation().getDegrees())); // X 14.7345

    public static final Pose2d kBlueClimbPoseFeed = new Pose2d(4.4, 3.3, Rotation2d.fromDegrees(60.0));
    public static final Pose2d kBlueClimbPoseAmp = new Pose2d(4.43, 4.95, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d kBlueClimbPoseMidline = new Pose2d(5.8, 4.1, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kRedClimbPoseFeed = new Pose2d(kFieldWidthMeters - kBlueClimbPoseFeed.getX(),
            kBlueClimbPoseFeed.getY(), Rotation2d.fromDegrees(180 - kBlueClimbPoseFeed.getRotation().getDegrees()));
    public static final Pose2d kRedClimbPoseAmp = new Pose2d(kFieldWidthMeters - kBlueClimbPoseAmp.getX(),
            kBlueClimbPoseAmp.getY(), Rotation2d.fromDegrees(180 - kBlueClimbPoseAmp.getRotation().getDegrees()));
    public static final Pose2d kRedClimbPoseMidline = new Pose2d(kFieldWidthMeters - kBlueClimbPoseMidline.getX(),
            kBlueClimbPoseMidline.getY(),
            Rotation2d.fromDegrees(180 - kBlueClimbPoseMidline.getRotation().getDegrees()));
    public static final Translation3d kRedSpeakerPose = new Translation3d(
            Constants.kAprilTagLayout.getTagPose(4).get().getX(),
            Constants.kAprilTagLayout.getTagPose(4).get().getY(), 2.045);
    public static final Translation3d kBlueSpeakerPose = new Translation3d(
            Constants.kAprilTagLayout.getTagPose(7).get().getX(),
            Constants.kAprilTagLayout.getTagPose(7).get().getY(), 2.045);
    public static final Translation3d kRedSpeakerTopPose = new Translation3d(
            Constants.kAprilTagLayout.getTagPose(4).get().getX() - Units.inchesToMeters(20.057),
            Constants.kAprilTagLayout.getTagPose(4).get().getY(),
            Constants.kAprilTagLayout.getTagPose(4).get().getZ() + Units.inchesToMeters(32.563));
    public static final Translation3d kBlueSpeakerTopPose = new Translation3d(
            Constants.kAprilTagLayout.getTagPose(7).get().getX() + Units.inchesToMeters(20.057),
            Constants.kAprilTagLayout.getTagPose(7).get().getY(),
            Constants.kAprilTagLayout.getTagPose(7).get().getZ() + Units.inchesToMeters(32.563));

    public static final Translation2d kBlueStageCenterPose = new Translation2d(4.83,
            kFieldWidthMeters / 2.0);
    public static final Translation2d kRedStageCenterPose = new Translation2d(
            kFieldLengthMeters - kBlueStageCenterPose.getX(),
            kBlueStageCenterPose.getY());
    public static final double kBlueSpeakerToStageAutoSwitchX = 7.0;
    public static final double kRedSpeakerToStageAutoSwitchX = kFieldLengthMeters - kBlueSpeakerToStageAutoSwitchX;

    public static final double kNoteReleaseHeight = Units.inchesToMeters(22.183);
    public static final Pose3d kLeftRedSpeakerPose = new Pose3d(Constants.kAprilTagLayout.getTagPose(4).get().getX(),
            5.875, 2.04, new Rotation3d());
    public static final Pose3d kRightRedSpeakerPose = new Pose3d(Constants.kAprilTagLayout.getTagPose(4).get().getX(),
            5.29, 2.04, new Rotation3d());
    public static final Pose3d kLeftBlueSpeakerPose = new Pose3d(Constants.kAprilTagLayout.getTagPose(7).get().getX(),
            5.875, 2.04, new Rotation3d());
    public static final Pose3d kRightBlueSpeakerPose = new Pose3d(Constants.kAprilTagLayout.getTagPose(7).get().getX(),
            5.29, 2.04, new Rotation3d());
    public static final double kSpeakerLengthMeters = kLeftRedSpeakerPose.getY() - kRightRedSpeakerPose.getY();

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.75;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.85;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 6.0;
        public static final double kPYController = 2.0;
        public static final double kPThetaController = 4.0;

        public static final double kTranslationKa = 0.0;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final PathConstraints kAutoAlignPathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared, kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);

        public static final Map<MidlineNote, List<MidlineNote>> kDefaultNoteToScoreToNoteMap = Map.ofEntries(
                Map.entry(MidlineNote.A, List.of(MidlineNote.B, MidlineNote.C)),
                Map.entry(MidlineNote.B, List.of(MidlineNote.A, MidlineNote.C)),
                Map.entry(MidlineNote.C, List.of(MidlineNote.B, MidlineNote.D)),
                Map.entry(MidlineNote.D, List.of(MidlineNote.E, MidlineNote.C)),
                Map.entry(MidlineNote.E, List.of(MidlineNote.D, MidlineNote.C)));

        public static final Map<MidlineNote, List<MidlineNote>> kACBNoteToScoreToNoteMap = Map.ofEntries(
                Map.entry(MidlineNote.A, List.of(MidlineNote.C, MidlineNote.B)),
                Map.entry(MidlineNote.B, List.of(MidlineNote.A, MidlineNote.C)),
                Map.entry(MidlineNote.C, List.of(MidlineNote.B, MidlineNote.D)),
                Map.entry(MidlineNote.D, List.of(MidlineNote.E, MidlineNote.C)),
                Map.entry(MidlineNote.E, List.of(MidlineNote.D, MidlineNote.C)));

        public static final Map<MidlineNote, List<MidlineNote>> kBCANoteToScoreToNoteMap = Map.ofEntries(
                Map.entry(MidlineNote.A, List.of(MidlineNote.C, MidlineNote.B)),
                Map.entry(MidlineNote.B, List.of(MidlineNote.C, MidlineNote.A)),
                Map.entry(MidlineNote.C, List.of(MidlineNote.A, MidlineNote.D)),
                Map.entry(MidlineNote.D, List.of(MidlineNote.E, MidlineNote.C)),
                Map.entry(MidlineNote.E, List.of(MidlineNote.D, MidlineNote.C)));

        public static final Map<MidlineNote, List<MidlineNote>> kCABNoteToScoreToNoteMap = Map.ofEntries(
                Map.entry(MidlineNote.A, List.of(MidlineNote.B, MidlineNote.C)),
                Map.entry(MidlineNote.B, List.of(MidlineNote.C, MidlineNote.A)),
                Map.entry(MidlineNote.C, List.of(MidlineNote.A, MidlineNote.D)),
                Map.entry(MidlineNote.D, List.of(MidlineNote.E, MidlineNote.C)),
                Map.entry(MidlineNote.E, List.of(MidlineNote.D, MidlineNote.C)));

        public static final Map<PriorityMidlineSequence, Map<MidlineNote, List<MidlineNote>>> kClosestNoteToScoreToNoteMap = Map
                .ofEntries(
                        Map.entry(PriorityMidlineSequence.ABC, kDefaultNoteToScoreToNoteMap),
                        Map.entry(PriorityMidlineSequence.ACB, kACBNoteToScoreToNoteMap),
                        Map.entry(PriorityMidlineSequence.BAC, kDefaultNoteToScoreToNoteMap),
                        Map.entry(PriorityMidlineSequence.BCA, kBCANoteToScoreToNoteMap),
                        Map.entry(PriorityMidlineSequence.CAB, kCABNoteToScoreToNoteMap),
                        Map.entry(PriorityMidlineSequence.CBA, kDefaultNoteToScoreToNoteMap),
                        Map.entry(PriorityMidlineSequence.EDC, kDefaultNoteToScoreToNoteMap),
                        Map.entry(PriorityMidlineSequence.DEC, kDefaultNoteToScoreToNoteMap));
    }

    public static final class LEDConstants {
        public static final CANDeviceId kCANdleId = new CANDeviceId(30, kCanBusCanivore);
        public static final int kNonCandleLEDCount = 30;
        public static final int kCandleLEDCount = 8;
        public static final int kMaxLEDCount = kNonCandleLEDCount + kCandleLEDCount;
    }

    /**
     * Check if this system has a certain mac address in any network device.
     *
     * @param mac_address Mac address to check.
     * @return true if some device with this mac address exists on this system.
     */
    public static boolean hasMacAddress(final String mac_address) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) {
                    continue;
                }
                StringBuilder device_mac_sb = new StringBuilder();
                System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String device_mac = device_mac_sb.toString();
                    System.out.println("hasMacAddress: NIS " + nis.getDisplayName() + " device_mac: " + device_mac);
                    if (mac_address.equals(device_mac)) {
                        System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
                        return true;
                    }
                } else {
                    System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
                }
            }

        } catch (SocketException e) {
            e.printStackTrace();
        }
        return false;
    }
}
