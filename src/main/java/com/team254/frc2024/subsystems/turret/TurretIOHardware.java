package com.team254.frc2024.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team254.frc2024.Constants;
import com.team254.frc2024.Robot;
import com.team254.lib.util.CTREUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.List;

/**
 * The {@code TurretIOHardware} class implements the {@link TurretIO} interface
 * and provides
 * hardware-level control and monitoring for the turret subsystem. It utilizes
 * the TalonFX
 * motor controller and CANCoder sensors to manage turret positioning and
 * movement.
 */
public class TurretIOHardware implements TurretIO {

    protected final TalonFX talon = new TalonFX(Constants.TurretConstants.kTurretTalonCanID.getDeviceNumber(),
            Constants.TurretConstants.kTurretTalonCanID.getBus());
    protected final CANcoder canCoder1To1 = new CANcoder(
            Constants.TurretConstants.kTurret1To1CANCoder.getDeviceNumber(),
            Constants.TurretConstants.kTurret1To1CANCoder.getBus());
    protected final CANcoder canCoder3To1 = new CANcoder(
            Constants.TurretConstants.kTurret3To1CANCoder.getDeviceNumber(),
            Constants.TurretConstants.kTurret3To1CANCoder.getBus());
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0).withSlot(0);
    private final StatusSignal<Double> positionSignal = talon.getPosition();
    private final StatusSignal<Double> velocitySignal = talon.getVelocity();
    private final StatusSignal<Double> voltsSignal = talon.getMotorVoltage();
    private final StatusSignal<Double> currentStatorSignal = talon.getStatorCurrent();
    private final StatusSignal<Double> currentSupplySignal = talon.getSupplyCurrent();
    private final StatusSignal<Double> cancoder1AbsolutePosition = canCoder1To1.getPosition();
    private final StatusSignal<Double> cancoder1Velocity = canCoder1To1.getVelocity();
    private boolean cancoderOffset = false;
    private final StatusSignal<Double> cancoder2AbsolutePosition = canCoder3To1.getPosition();

    public TurretIOHardware() {

        var cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = -1.0 * Constants.TurretConstants.k1To1TurretCancoderOffset;
        CTREUtil.applyConfiguration(canCoder1To1, cancoderConfig);

        cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = -1.0 * Constants.TurretConstants.k3To1TurretCancoderOffset;
        CTREUtil.applyConfiguration(canCoder3To1, cancoderConfig);
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.0 / Constants.TurretConstants.kTurretGearRatio;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1.0 / Constants.TurretConstants.kTurretGearRatio;

        if (Robot.isReal()) {
            config.CurrentLimits.StatorCurrentLimit = 150.0;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.ClosedLoopRamps = Constants.makeDefaultClosedLoopRampConfig();
            config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
            config.OpenLoopRamps = Constants.makeDefaultOpenLoopRampConfig();
        }

        config.Slot0.kS = 0.18;
        config.Slot0.kP = 6.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kV = 0.120;
        config.Slot0.kA = 0.0001 * 12.0;

        config.MotionMagic.MotionMagicJerk = 0.0;
        config.MotionMagic.MotionMagicAcceleration = 900.0;
        config.MotionMagic.MotionMagicCruiseVelocity = 90.0;
        CTREUtil.applyConfiguration(talon, config);
        BaseStatusSignal.setUpdateFrequencyForAll(50, voltsSignal, currentStatorSignal, currentSupplySignal,
                cancoder2AbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(250, cancoder1AbsolutePosition, cancoder1Velocity, positionSignal,
                velocitySignal);
        talon.optimizeBusUtilization();
    }

    public List<BaseStatusSignal> getStatusSignals() {
        // Only read position and velocity at 250 hz
        return Arrays.asList(positionSignal, velocitySignal, cancoder1AbsolutePosition, cancoder1Velocity);
    }

    public void readFastInputs(FastTurretInputs inputs) {
        double talonPosition = BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal);
        double kGearRatio = Constants.TurretConstants.kTurretGearRatio;
        inputs.positionRad = Units.rotationsToRadians(talonPosition * kGearRatio);
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocitySignal.getValueAsDouble() * kGearRatio);
        inputs.turretPositionAbsolute = Rotation2d
                .fromRotations(
                        BaseStatusSignal.getLatencyCompensatedValue(cancoder1AbsolutePosition, cancoder1Velocity));
    }

    @Override
    public void readInputs(TurretInputs inputs) {
        BaseStatusSignal.refreshAll(voltsSignal, currentStatorSignal, currentSupplySignal,
                cancoder2AbsolutePosition);
        if (!cancoderOffset && cancoder1AbsolutePosition != null) {
            talon.setPosition(getTurretAngleOffset());
            cancoderOffset = true;
        }
        Logger.recordOutput("Turret/IO/cancoderOffset", cancoderOffset);
        inputs.cancoder1AbsolutePosition = cancoder1AbsolutePosition.getValueAsDouble();
        inputs.cancoder2AbsolutePosition = cancoder2AbsolutePosition.getValueAsDouble();
        inputs.appliedVolts = voltsSignal.getValueAsDouble();
        inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
        inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        talon.setControl(dutyCycleControl.withOutput(dutyCycle));
        Logger.recordOutput("Turret/IO/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
    }

    private double getTurretAngleOffset() {
        BaseStatusSignal.waitForAll(10.0, cancoder1AbsolutePosition, cancoder2AbsolutePosition);
        double cancoder1To1 = cancoder1AbsolutePosition.getValueAsDouble();
        // Make cancoder3to1 read true value.
        double cancoder3To1 = cancoder2AbsolutePosition.getValueAsDouble() * (42.0 / 18.0);

        double offset = cancoder3To1 - cancoder1To1;
        while (Math.abs(offset) > 0.5) {
            if (offset > 0.5) {
                cancoder1To1 += 1.0;
            } else if (offset < -0.5) {
                cancoder1To1 -= 1.0;
            }
            offset = cancoder3To1 - cancoder1To1;
        }
        return cancoder1To1 / Constants.TurretConstants.kTurretGearRatio;
    }

    @Override
    public void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {
        double setpointRadians = MathUtil.clamp(
                radiansFromCenter, Constants.TurretConstants.kTurretMinPositionRadians,
                Constants.TurretConstants.kTurretMaxPositionRadians);
        double setpointRotations = Units.radiansToRotations(setpointRadians);
        double setpointRotor = setpointRotations / Constants.TurretConstants.kTurretGearRatio;
        double ffVel = Units.radiansToRotations(radsPerSecond) / Constants.TurretConstants.kTurretGearRatio;
        talon.setControl(positionVoltageControl.withPosition(setpointRotor).withVelocity(ffVel));
        Logger.recordOutput("Turret/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
        Logger.recordOutput("Turret/IO/setPositionSetpoint/radsPerSecond", radsPerSecond);
        Logger.recordOutput("Turret/IO/setPositionSetpoint/ffVel", ffVel);
        Logger.recordOutput("Turret/IO/setPositionSetpoint/setpointRotor", setpointRotor);
        Logger.recordOutput("Turret/IO/setPositionSetpoint/radsPerSecondRotor",
                radsPerSecond / Constants.TurretConstants.kTurretGearRatio);
    }
}
