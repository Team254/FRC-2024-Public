package com.team254.lib.util;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class SpeakerTargetFactory {

    static InterpolatingTreeMap<Double, Double> heightMap = new InterpolatingTreeMap<Double, Double>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static {
        heightMap.put(1.5, 0.08);
        heightMap.put(1.8, 0.04);
        heightMap.put(2.6, 0.02);
        heightMap.put(4.0, 0.0);
    }
    static InterpolatingTreeMap<Double, Double> distanceOffsetMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static {
        distanceOffsetMap.put(1.4, Units.inchesToMeters(12.0));
        distanceOffsetMap.put(3.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(6.0);

    public static Translation3d generate(RobotState robotState) {
        // uncomment to calibrate shooter to center of the face above the goal
        // return robotState.isRedAlliance() ? Constants.kRedSpeakerTopPose :
        // Constants.kBlueSpeakerTopPose;

        var speakerPose = robotState.isRedAlliance() ? Constants.kRedSpeakerPose
                : Constants.kBlueSpeakerPose;

        double distance = new Translation2d(speakerPose.getX(), speakerPose.getY()).getDistance(
                robotState.getLatestFieldToRobot().getValue().getTranslation());

        double distanceOffset = distanceOffsetMap.get(distance);
        // Do math in blue alliance, we flip for red.
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (robotState.isRedAlliance()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        Logger.recordOutput("SpeakerTargetFactory/distanceFromTarget", distance);
        speakerPose = new Translation3d(
                speakerPose.getX() + offSet.getX(), speakerPose.getY() + offSet.getY(),
                speakerPose.getZ() + heightMap.get(distance));
        Logger.recordOutput("targetPose", speakerPose);
        Logger.recordOutput("targetPose2d", new Translation2d(speakerPose.getX(), speakerPose.getY()));
        return speakerPose;
    }
}
