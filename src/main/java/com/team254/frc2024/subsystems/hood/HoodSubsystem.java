package com.team254.frc2024.subsystems.hood;

import com.team254.frc2024.RobotState;
import com.team254.lib.time.RobotTime;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Subsystem class for controlling the hood mechanism.
 * The {@code HoodSubsystem} interacts with the HoodIO interface to manage hood
 * position,
 * velocity, and duty cycle outputs. It integrates with the RobotState to track
 * the
 * hood's current position and rotation, and provides several commands for
 * adjusting
 * the hood setpoint and running the hood motor in different modes.
 * <p>
 * This class also logs hood data, such as setpoints and latency, to assist
 * with debugging and performance tracking during operation.
 * </p>
 */

public class HoodSubsystem extends SubsystemBase {

    private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
    private RobotState robotState;

    private final HoodIO io;

    private double hoodSetpointRadiansFromCenter = 0.0;

    public HoodSubsystem(final HoodIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;
    }

    public void setTeleopDefaultCommand() {
        this.setDefaultCommand(run(() -> {
            setPositionSetpointImpl(hoodSetpointRadiansFromCenter, 0.0);
        }).withName("Hood Maintain Setpoint (default)"));
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.readInputs(inputs);
        Logger.processInputs("Hood", inputs);
        io.update(inputs);

        robotState.addHoodRotation(new Rotation2d(inputs.positionRad));
        Logger.recordOutput("Hood/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    public Command moveHoodSetpoint(DoubleSupplier radiansFromCenter) {
        return runOnce(() -> {
            hoodSetpointRadiansFromCenter = radiansFromCenter.getAsDouble();
        });
    }

    public Command runDutyCycle(DoubleSupplier percentOutput) {
        return startEnd(
                () -> setDutyCycleOut(percentOutput.getAsDouble()),
                () -> setDutyCycleOut(0.0)).withName("Hood DutyCycleControl");
    }

    private void setPositionSetpointImpl(double radiansFromCenter, double radsPerSec) {
        Logger.recordOutput("Hood/API/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
        Logger.recordOutput("Hood/API/setPositionSetpoint/radsPerSec", radsPerSec);
        io.setPositionSetpoint(radiansFromCenter, radsPerSec);
    }

    private void setDutyCycleOut(double percentOutput) {
        io.setDutyCycleOut(percentOutput);
    }

    public Command positionSetpointCommand(DoubleSupplier radiansFromCenter, DoubleSupplier radsPerSec) {
        return run(() -> {
            double setpoint = radiansFromCenter.getAsDouble();
            setPositionSetpointImpl(setpoint, radsPerSec.getAsDouble());
            hoodSetpointRadiansFromCenter = setpoint;
        }).withName("Hood positionSetpointCommand");
    }

    public double getSetpoint() {
        return hoodSetpointRadiansFromCenter;
    }

    public double getCurrentPosition() {
        return inputs.positionRad;
    }

    public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
        return new WaitUntilCommand(() -> {
            return Math.abs(new Rotation2d(inputs.positionRad).rotateBy(
                    new Rotation2d(radiansFromCenter.getAsDouble()).unaryMinus()).getRadians()) < toleranceRadians;
        }).withName("Hood wait for position");
    }

    public double getCurrentPositionRotations() {
        return inputs.positionRotations;
    }

    public void resetZeroPoint() {
        io.resetZeroPoint();
    }
}
