/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.team254.lib.ctre.swerve.utility;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * Phoenix-centric PID controller taken from WPI's
 * {@link edu.wpi.first.math.controller.PIDController} class.
 * <p>
 * This class differs from the WPI implementation by using explicit timestamps
 * for
 * integral/derivative calculations. Ideally, these timestamps come from the
 * {@link com.ctre.phoenix6.StatusSignal}.
 */
public class PhoenixPIDController implements Sendable, AutoCloseable {
    private static int instances;

    // Factor for "proportional" control
    private double m_kp;

    // Factor for "integral" control
    private double m_ki;

    // Factor for "derivative" control
    private double m_kd;

    private double m_maximumIntegral = 1.0;

    private double m_minimumIntegral = -1.0;

    private double m_maximumInput;

    private double m_minimumInput;

    // Do the endpoints wrap around? e.g. Absolute encoder
    private boolean m_continuous;

    // The error at the time of the most recent call to calculate()
    private double m_positionError;
    private double m_velocityError;

    // The error at the time of the second-most-recent call to calculate() (used to
    // compute velocity)
    private double m_prevError;

    // The sum of the errors for use in the integral calc
    private double m_totalError;

    // The error that is considered at setpoint.
    private double m_positionTolerance = 0.05;
    private double m_velocityTolerance = Double.POSITIVE_INFINITY;

    private double m_setpoint;
    private double m_measurement;

    private boolean m_haveMeasurement;
    private boolean m_haveSetpoint;

    private double m_lastAppliedOutput;

    // The last timestamp acquired when performing a calculation
    private double m_lastTimestamp;

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public PhoenixPIDController(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;

        instances++;
        SendableRegistry.addLW(this, "PIDController", instances);

        MathSharedStore.reportUsage(MathUsageId.kController_PIDController2, instances);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>
     * Set the proportional, integral, and differential coefficients.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public void setPID(double kp, double ki, double kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    /**
     * Sets the Proportional coefficient of the PID controller gain.
     *
     * @param kp proportional coefficient
     */
    public void setP(double kp) {
        m_kp = kp;
    }

    /**
     * Sets the Integral coefficient of the PID controller gain.
     *
     * @param ki integral coefficient
     */
    public void setI(double ki) {
        m_ki = ki;
    }

    /**
     * Sets the Differential coefficient of the PID controller gain.
     *
     * @param kd differential coefficient
     */
    public void setD(double kd) {
        m_kd = kd;
    }

    /**
     * Get the Proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_kp;
    }

    /**
     * Get the Integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_ki;
    }

    /**
     * Get the Differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_kd;
    }

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    public double getPositionTolerance() {
        return m_positionTolerance;
    }

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    public double getVelocityTolerance() {
        return m_velocityTolerance;
    }

    /**
     * Returns the current setpoint of the PIDController.
     *
     * @return The current setpoint.
     */
    public double getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the setpoint.
     *
     * <p>
     * This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetpoint() {
        return m_haveMeasurement
                && m_haveSetpoint
                && Math.abs(m_positionError) < m_positionTolerance
                && Math.abs(m_velocityError) < m_velocityTolerance;
    }

    /**
     * Enables continuous input.
     *
     * <p>
     * Rather then using the max and min input range as constraints, it considers
     * them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_continuous = true;
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        m_continuous = false;
    }

    /**
     * Returns true if continuous input is enabled.
     *
     * @return True if continuous input is enabled.
     */
    public boolean isContinuousInputEnabled() {
        return m_continuous;
    }

    /**
     * Sets the minimum and maximum values for the integrator.
     *
     * <p>
     * When the cap is reached, the integrator value is added to the controller
     * output rather than
     * the integrator value times the integral gain.
     *
     * @param minimumIntegral The minimum value of the integrator.
     * @param maximumIntegral The maximum value of the integrator.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_minimumIntegral = minimumIntegral;
        m_maximumIntegral = maximumIntegral;
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return m_positionError;
    }

    /**
     * Returns the velocity error.
     *
     * @return The velocity error.
     */
    public double getVelocityError() {
        return m_velocityError;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement      The current measurement of the process variable.
     * @param setpoint         The setpoint to target
     * @param currentTimestamp The current timestamp to use for calculating
     *                         integral/derivative error
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint, double currentTimestamp) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        m_measurement = measurement;
        m_prevError = m_positionError;
        m_haveMeasurement = true;

        double thisPeriod = currentTimestamp - m_lastTimestamp;
        m_lastTimestamp = currentTimestamp;

        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_positionError = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        } else {
            m_positionError = m_setpoint - m_measurement;
        }

        m_velocityError = (m_positionError - m_prevError) / thisPeriod;

        if (m_ki != 0) {
            m_totalError = MathUtil.clamp(
                    m_totalError + m_positionError * thisPeriod,
                    m_minimumIntegral / m_ki,
                    m_maximumIntegral / m_ki);
        }

        m_lastAppliedOutput = m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;
        return m_lastAppliedOutput;
    }

    /* Get the last applied output from this PID controller */
    public double getLastAppliedOutput() {
        return m_lastAppliedOutput;
    }

    /** Resets the previous error and the integral term. */
    public void reset() {
        m_positionError = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_velocityError = 0;
        m_haveMeasurement = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("setpoint", this::getSetpoint, null);
    }
}
