/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.team254.lib.ctre.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Extremely simplified swerve drive simulation class.
 * <p>
 * This class assumes that the swerve drive is perfect, meaning
 * that there is no scrub and the wheels do not slip.
 * <p>
 * In addition, it assumes the inertia of the robot is governed only
 * by the inertia of the steer module and the individual drive wheels.
 * Robot-wide inertia is not accounted for, and neither is translational
 * vs rotational inertia of the robot.
 * <p>
 * These assumptions provide a simplified example that can demonstrate the
 * behavior of a swerve drive in simulation. Users are encouraged to
 * expand this model for their own use.
 */
public class SimSwerveDrivetrain {
    public class SimSwerveModule {
        /** Reference to motor simulation for the steer motor */
        public final DCMotorSim SteerMotor;
        /** Reference to motor simulation for drive motor */
        public final DCMotorSim DriveMotor;
        /** Reference to steer gearing for updating CANcoder */
        public final double SteerGearing;
        /** Reference to steer gearing for updating CANcoder */
        public final double DriveGearing;
        /** Voltage necessary for the steer motor to overcome friction */
        public final double SteerFrictionVoltage;
        /** Voltage necessary for the drive motor to overcome friction */
        public final double DriveFrictionVoltage;
        /** Whether the steer motor is inverted */
        public final boolean SteerMotorInverted;
        /** Whether the drive motor is inverted */
        public final boolean DriveMotorInverted;

        public SimSwerveModule(double steerGearing, double steerInertia, double steerFrictionVoltage,
                boolean steerMotorInverted,
                double driveGearing, double driveInertia, double driveFrictionVoltage, boolean driveMotorInverted) {
            SteerMotor = new DCMotorSim(DCMotor.getFalcon500(1), steerGearing, steerInertia);
            DriveMotor = new DCMotorSim(DCMotor.getFalcon500(1), driveGearing, driveInertia);
            SteerGearing = steerGearing;
            DriveGearing = driveGearing;
            SteerFrictionVoltage = steerFrictionVoltage;
            DriveFrictionVoltage = driveFrictionVoltage;
            SteerMotorInverted = steerMotorInverted;
            DriveMotorInverted = driveMotorInverted;
        }
    }

    public final Pigeon2SimState PigeonSim;
    protected final SimSwerveModule[] m_modules;
    protected final int ModuleCount;
    public final SwerveDriveKinematics Kinem;
    public Rotation2d LastAngle = new Rotation2d();

    public SimSwerveDrivetrain(Translation2d[] wheelLocations,
            Pigeon2 pigeon,
            SwerveDrivetrainConstants driveConstants,
            SwerveModuleConstants... moduleConstants) {
        PigeonSim = pigeon.getSimState();
        ModuleCount = moduleConstants.length;
        m_modules = new SimSwerveModule[ModuleCount];
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i] = new SimSwerveModule(moduleConstants[i].SteerMotorGearRatio, moduleConstants[i].SteerInertia,
                    moduleConstants[i].SteerFrictionVoltage, moduleConstants[i].SteerMotorInverted,
                    moduleConstants[i].DriveMotorGearRatio, moduleConstants[i].DriveInertia,
                    moduleConstants[i].DriveFrictionVoltage, moduleConstants[i].DriveMotorInverted);
        }

        Kinem = new SwerveDriveKinematics(wheelLocations);
    }

    /**
     * Update this simulation for the time duration.
     * <p>
     * This performs a simulation update on all the simulated devices
     *
     * @param dtSeconds      The time delta between this update and the previous
     *                       update
     * @param supplyVoltage  The voltage as seen at the motor controllers
     * @param modulesToApply What modules to apply the update to
     */
    public void update(double dtSeconds, double supplyVoltage, SwerveModule... modulesToApply) {
        if (m_modules.length != ModuleCount)
            return;

        SwerveModuleState[] states = new SwerveModuleState[ModuleCount];
        /* Update our sim devices */
        for (int i = 0; i < ModuleCount; ++i) {
            TalonFXSimState steerMotor = modulesToApply[i].getSteerMotor().getSimState();
            TalonFXSimState driveMotor = modulesToApply[i].getDriveMotor().getSimState();
            CANcoderSimState cancoder = modulesToApply[i].getCANcoder().getSimState();

            steerMotor.Orientation = m_modules[i].SteerMotorInverted ? ChassisReference.Clockwise_Positive
                    : ChassisReference.CounterClockwise_Positive;
            driveMotor.Orientation = m_modules[i].DriveMotorInverted ? ChassisReference.Clockwise_Positive
                    : ChassisReference.CounterClockwise_Positive;

            steerMotor.setSupplyVoltage(supplyVoltage);
            driveMotor.setSupplyVoltage(supplyVoltage);
            cancoder.setSupplyVoltage(supplyVoltage);

            m_modules[i].SteerMotor
                    .setInputVoltage(addFriction(steerMotor.getMotorVoltage(), m_modules[i].SteerFrictionVoltage));
            m_modules[i].DriveMotor
                    .setInputVoltage(addFriction(driveMotor.getMotorVoltage(), m_modules[i].DriveFrictionVoltage));

            m_modules[i].SteerMotor.update(dtSeconds);
            m_modules[i].DriveMotor.update(dtSeconds);

            steerMotor.setRawRotorPosition(
                    m_modules[i].SteerMotor.getAngularPositionRotations() * m_modules[i].SteerGearing);
            steerMotor.setRotorVelocity(
                    m_modules[i].SteerMotor.getAngularVelocityRPM() / 60.0 * m_modules[i].SteerGearing);

            /* CANcoders see the mechanism, so don't account for the steer gearing */
            cancoder.setRawPosition(m_modules[i].SteerMotor.getAngularPositionRotations());
            cancoder.setVelocity(m_modules[i].SteerMotor.getAngularVelocityRPM() / 60.0);

            driveMotor.setRawRotorPosition(
                    m_modules[i].DriveMotor.getAngularPositionRotations() * m_modules[i].DriveGearing);
            driveMotor.setRotorVelocity(
                    m_modules[i].DriveMotor.getAngularVelocityRPM() / 60.0 * m_modules[i].DriveGearing);

            states[i] = modulesToApply[i].getCurrentState();
        }

        double angleChange = Kinem.toChassisSpeeds(states).omegaRadiansPerSecond * dtSeconds;
        LastAngle = LastAngle.plus(Rotation2d.fromRadians(angleChange));
        PigeonSim.setRawYaw(LastAngle.getDegrees());
    }

    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage    Voltage output by the motor
     * @param frictionVoltage Voltage required to overcome friction
     * @return Friction-dampened motor voltage
     */
    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }
}
