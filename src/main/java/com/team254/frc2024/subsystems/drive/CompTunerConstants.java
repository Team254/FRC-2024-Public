package com.team254.frc2024.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team254.lib.ctre.swerve.SwerveModule.ClosedLoopOutputType;
import com.team254.lib.ctre.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.team254.frc2024.Constants;
import com.team254.lib.ctre.swerve.SwerveDrivetrainConstants;
import com.team254.lib.ctre.swerve.SwerveModuleConstants;
import com.team254.lib.ctre.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class CompTunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.35).withKI(0).withKD(0.0)
            .withKS(0).withKV(12.0 / 88.2142857143).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 80.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = Constants.DriveConstants.kDriveMaxSpeed;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.125 * 14.0 / 13.0;

    private static final double kDriveGearRatio = (50.0 / 13.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 1.95;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "canivore";
    private static final int kPigeonId = 20;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final TalonFXConfiguration initialDriveConfigs = new TalonFXConfiguration();
    static {
        initialDriveConfigs.CurrentLimits.SupplyCurrentLimit = 80;
        initialDriveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        initialDriveConfigs.Audio.BeepOnBoot = false;
        initialDriveConfigs.Audio.BeepOnConfig = false;
        initialDriveConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01;
        initialDriveConfigs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.01;
        initialDriveConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
    }
    private static final TalonFXConfiguration initialSteerConfigs = new TalonFXConfiguration();
    static {
        initialSteerConfigs.Audio.BeepOnBoot = false;
        initialSteerConfigs.Audio.BeepOnConfig = false;
        initialSteerConfigs.CurrentLimits.StatorCurrentLimit = 50.0;
        initialSteerConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
    }
    private static final TalonFXConfiguration simInitialDriveConfigs = initialDriveConfigs;
    static {
        simInitialDriveConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        simInitialDriveConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
    }
    private static final TalonFXConfiguration simInitialSteerConfigs = initialSteerConfigs;
    static {
        simInitialSteerConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        simInitialSteerConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
    }

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed)
            .withDriveMotorInitialConfigs(
                    RobotBase.isSimulation() ? simInitialDriveConfigs : initialDriveConfigs)
            .withSteerMotorInitialConfigs(
                    RobotBase.isSimulation() ? simInitialSteerConfigs : initialSteerConfigs);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 4;
    private static final int kFrontLeftEncoderId = 10;
    private static final double kFrontLeftEncoderOffset = -0.320800;

    private static final double kFrontLeftXPosInches = 10.875;
    private static final double kFrontLeftYPosInches = 10.875;

    // Front Right
    private static final int kFrontRightDriveMotorId = 1;
    private static final int kFrontRightSteerMotorId = 2;
    private static final int kFrontRightEncoderId = 9;
    private static final double kFrontRightEncoderOffset = -0.019287;

    private static final double kFrontRightXPosInches = 10.875;
    private static final double kFrontRightYPosInches = -10.875;

    // Back Left
    private static final int kBackLeftDriveMotorId = 5;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 11;
    private static final double kBackLeftEncoderOffset = -0.057129;

    private static final double kBackLeftXPosInches = -10.875;
    private static final double kBackLeftYPosInches = 10.875;

    // Back Right
    private static final int kBackRightDriveMotorId = 7;
    private static final int kBackRightSteerMotorId = 8;
    private static final int kBackRightEncoderId = 12;
    private static final double kBackRightEncoderOffset = -0.299316;

    private static final double kBackRightXPosInches = -10.875;
    private static final double kBackRightYPosInches = -10.875;

    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
            kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants,
            FrontLeft,
            FrontRight, BackLeft, BackRight);
}
