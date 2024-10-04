package com.team254.frc2024.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team254.frc2024.Constants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.RobotState;
import com.team254.lib.loops.IStatusSignalLoop;
import com.team254.lib.subsystems.MotorIO;
import com.team254.lib.subsystems.MotorInputsAutoLogged;
import com.team254.lib.subsystems.ServoMotorSubsystem;
import com.team254.lib.subsystems.ServoMotorSubsystemConfig;

/**
 * Subsystem class for controlling the intake mechanism of the FRC 2024 robot.
 * The IntakeSubsystem manages both motor control for the intake and sensor
 * feedback for detecting
 * game pieces. It integrates with the robot's state and logs important data for
 * diagnostics and performance monitoring.
 * The class extends the ServoMotorSubsystem to control the intake motor and
 * implements the IStatusSignalLoop interface
 * to handle sensor status updates.
 */
public class IntakeSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> implements IStatusSignalLoop {
    private IntakeSensorInputsAutoLogged inputsSensors = new IntakeSensorInputsAutoLogged();
    private IntakeSensorIO ioSensors;
    private AtomicBoolean bannerHasPiece = new AtomicBoolean(false);

    private final RobotState state;

    private double timeOfLastBanner = 0.0;
    private Debouncer bannerDebounce = new Debouncer(Constants.SensorConstants.kIntakeDebounceTime,
            Debouncer.DebounceType.kRising);

    public IntakeSubsystem(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO,
            final IntakeSensorIO sensorIO, RobotState robotState) {
        super(motorConfig, new MotorInputsAutoLogged(), motorIO);
        this.ioSensors = sensorIO;
        this.state = robotState;
    }

    @Override
    public void periodic() {
        super.periodic();
        ioSensors.readInputs(inputsSensors);
        inputsSensors.intakeBannerHasPiece = hasNoteAtIntakeBanner();
        state.updateLastTriggeredIntakeSensorTimestamp(inputsSensors.intakeBannerHasPiece);
        Logger.recordOutput("Intake/lastTriggeredBannerSensorTimestamp", state.lastTriggeredIntakeSensorTimestamp());
        Logger.processInputs(getName() + "/sensors", inputsSensors);
        if (inputsSensors.intakeBannerHasPiece) {
            timeOfLastBanner = Timer.getFPGATimestamp();
        }
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Zero intake Duty Cycle"));
    }

    public boolean hasNoteAtIntakeBanner() {
        return bannerHasPiece.get();
    }

    public boolean hadNoteAtIntakeBanner(double secondsAgo) {
        return Timer.getFPGATimestamp() - secondsAgo <= timeOfLastBanner;
    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return new ArrayList<>();
    }

    @Override
    public void onLoop() {
        bannerHasPiece.set(bannerDebounce.calculate(ioSensors.getIntakeBanner().get()));
    }
}