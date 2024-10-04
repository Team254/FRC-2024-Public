package com.team254.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MathHelpers {
    public static final Pose2d kPose2dZero = new Pose2d();

    public static final Pose2d pose2dFromRotation(Rotation2d rotation) {
        return new Pose2d(kTranslation2dZero, rotation);
    }

    public static final Pose2d pose2dFromTranslation(Translation2d translation) {
        return new Pose2d(translation, kRotation2dZero);
    }

    public static final Rotation2d kRotation2dZero = new Rotation2d();
    public static final Rotation2d kRotation2dPi = Rotation2d.fromDegrees(180.0);

    public static final Translation2d kTranslation2dZero = new Translation2d();

    public static final Transform2d kTransform2dZero = new Transform2d();

    public static final Transform2d transform2dFromRotation(Rotation2d rotation) {
        return new Transform2d(kTranslation2dZero, rotation);
    }

    public static final Transform2d transform2dFromTranslation(Translation2d translation) {
        return new Transform2d(translation, kRotation2dZero);
    }
}
