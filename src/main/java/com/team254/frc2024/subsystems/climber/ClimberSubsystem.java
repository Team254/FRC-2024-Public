package com.team254.frc2024.subsystems.climber;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotState;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.DoubleSupplier;

/**
 * The ClimberSubsystem class is responsible for controlling the robot's climber
 * mechanism.
 * It manages motor positions, coordinates with the robot's state, and allows
 * for precise
 * movement control using motion magic and position feedback. This subsystem
 * includes commands
 * for setting motor positions, moving the climber slowly via duty cycle to a
 * certain position,
 * and zeroing the position after a climb.
 */

public class ClimberSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private RobotState robotState;
    private double positionCmd;

    public ClimberSubsystem(ServoMotorSubsystemConfig c, final MotorIO io, RobotState robotState) {
        super(c, new MotorInputsAutoLogged(), io);
        this.robotState = robotState;
        setCurrentPositionAsZero();
        setDefaultCommand(Commands.runOnce(() -> {
            positionCmd = inputs.unitPosition;
        }).andThen(positionSetpointCommand(() -> positionCmd)));
    }

    @Override
    public void periodic() {
        super.periodic();
        robotState.setClimberRotations(inputs.unitPosition);
    }

    public Command motionMagicSetpointCommandBlocking(double setpoint, double tolerance) {
        return motionMagicSetpointCommand(() -> setpoint)
                .until(() -> Util.epsilonEquals(getCurrentPosition(), setpoint, tolerance));
    }

    public Command jogAndThenZero() {
        return new ParallelCommandGroup(
                withoutLimitsTemporarily(),
                dutyCycleCommand(() -> -0.1)).finallyDo(() -> {
                    setCurrentPositionAsZero();
                });
    }

    public Command waitForClimberPosition(DoubleSupplier targetPosition) {
        return new WaitUntilCommand(() -> Util.epsilonEquals(inputs.unitPosition, targetPosition.getAsDouble(),
                Constants.ClimberConstants.kClimbClimbedPositionToleranceRotations));
    }
}
