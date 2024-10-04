package com.team254.frc2024.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.team254.frc2024.RobotState;
import com.team254.frc2024.simulation.SimulatedRobotState;
import com.team254.lib.ctre.swerve.SwerveDrivetrainConstants;
import com.team254.lib.ctre.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class DriveIOSim extends DriveIOHardware {

    private SimulatedRobotState simRobotState = null;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    Pose2d lastConsumedPose = null;
    Consumer<SwerveDriveState> simTelemetryConsumer = swerveDriveState -> {
        // First, Update the underlying telemetry consumer with the raw data. This kee
        telemetryConsumer_.accept(swerveDriveState);

        // Protect at init
        if (simRobotState == null) {
            return;
        }

        simRobotState.addFieldToRobot(swerveDriveState.Pose);
    };

    public DriveIOSim(RobotState robotState, SimulatedRobotState simRobotState,
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(robotState, driveTrainConstants, modules);
        this.simRobotState = simRobotState;

        // Rewrite the telemetry consumer with a consumer for sim
        registerTelemetry(simTelemetryConsumer);
        startSimThread();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void readInputs(DriveIOInputs inputs) {
        super.readInputs(inputs);

        // Handle the viz
        var pose = simRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", simRobotState.getLatestFieldToRobot());
        }
    }
}
