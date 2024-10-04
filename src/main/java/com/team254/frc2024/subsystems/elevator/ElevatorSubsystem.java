package com.team254.frc2024.subsystems.elevator;

import com.team254.frc2024.RobotState;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;

/**
 * The ElevatorSubsystem class is responsible for controlling the robot's
 * elevator mechanism.
 * It extends the ServoMotorSubsystem and uses motor inputs to manage and adjust
 * the elevator's
 * position, interfacing with the RobotState to update the robot's elevator
 * height in real time.
 * This class provides commands for maintaining position setpoints and blocking
 * motion until
 * the desired position is reached using motion magic. The default command
 * ensures the elevator
 * holds its current setpoint.
 */

public class ElevatorSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    protected RobotState robotState;

    public ElevatorSubsystem(ServoMotorSubsystemConfig c, final MotorIO io, RobotState robotState) {
        super(c, new MotorInputsAutoLogged(), io);
        this.robotState = robotState;
        setCurrentPositionAsZero();
        this.setDefaultCommand(
                motionMagicSetpointCommand(() -> getPositionSetpoint())
                        .withName("Elevator Maintain Setpoint (default)"));
    }

    @Override
    public void periodic() {
        super.periodic();
        robotState.setElevatorHeight(Units.inchesToMeters(inputs.unitPosition));
    }

    public Command motionMagicSetpointCommandBlocking(double setpoint, double tolerance) {
        return motionMagicSetpointCommand(() -> setpoint)
                .until(() -> Util.epsilonEquals(getCurrentPosition(), setpoint, tolerance));
    }
}
