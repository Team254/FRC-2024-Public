package com.team254.frc2024.subsystems.drive;

import com.team254.lib.ctre.swerve.SwerveDrivetrain;
import com.team254.lib.util.MathHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class DriveViz {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */

    private final Field2d field = new Field2d();

    public DriveViz(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SmartDashboard.putData("Field", field);
    }

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d lastPose = MathHelpers.kPose2dZero;
    private double lastTime = Logger.getTimestamp();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
            moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
            moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDrivetrain.SwerveDriveState state) {
        if (state == null || state.Pose == null || state.ModuleStates == null) {
            return;
        }

        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        Logger.recordOutput("Drive/Viz/Pose", pose);

        Pose3d pose3d = new Pose3d(pose.getX(), pose.getY(), 0.0,
                new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
        Logger.recordOutput("Drive/Viz/Pose3d", pose3d);

        field.setRobotPose(pose);

        /* Telemeterize the robot's general speeds */
        double currentTime = Logger.getTimestamp();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
        lastPose = pose;
        Translation2d velocities = distanceDiff.div(diffTime);

        Logger.recordOutput("Drive/Viz/Speed", velocities.getNorm());
        Logger.recordOutput("Drive/Viz/VelocityX", velocities.getX());
        Logger.recordOutput("Drive/Viz/VelocityY", velocities.getY());
        Logger.recordOutput("Drive/Viz/OdomPeriod", state.OdometryPeriod);

        /* Telemeterize the module's states */
        for (int i = 0; i < 4; ++i) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
            // Logger.recordOutput("Drive/Viz/Module_" + i, moduleMechanisms[i]);
        }
    }
}
