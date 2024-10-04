package com.team254.lib.util;

import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class PoopTargetFactory {

    public enum NearTarget {
        SHALLOW(kDefaultXSafetyMargin, Constants.kFieldWidthMeters - kPrimaryYOffsetFromAmpWall),
        DEEP_AMP(kDefaultXSafetyMargin, Constants.kFieldWidthMeters - kSecondaryYOffsetFromAmpWall);

        private double targetX, targetY;

        private NearTarget(double targetX, double targetY) {
            this.targetX = targetX;
            this.targetY = targetY;
        }

        public double getTargetX() {
            return targetX;
        }

        public double getTargetY() {
            return targetY;
        }
    }

    // All are BLUE primary.
    final static double kCloseWingX = Units.inchesToMeters(231.2);
    final static double kFarWingX = Constants.kFieldLengthMeters - Units.inchesToMeters(231.2);
    final static double kFarWingPoopBuffer = Units.inchesToMeters(72); // popsitive = further away from driver
    final static double kDefaultXSafetyMargin = Units.inchesToMeters(48);
    final static double kFarXSafetyMargin = Units.inchesToMeters(60);
    final static double kPrimaryYOffsetFromAmpWall = Units.inchesToMeters(72);
    final static double kSecondaryYOffsetFromAmpWall = Units.inchesToMeters(20);

    final static double kNominalPoopHeight = Constants.ShooterConstants.kPoopMaxApexHeight;
    final static double kLineDrivePoopHeight = Units.inchesToMeters(36.0);

    public static Translation3d primaryForFarZone() {
        // Aim at the wing line plus some margin in X
        final double targetX = kCloseWingX + kFarXSafetyMargin;
        // Aim at the amp wall plus some margin in Y
        final double targetY = Constants.kFieldWidthMeters - kPrimaryYOffsetFromAmpWall;

        return new Translation3d(targetX, targetY, kNominalPoopHeight);
    }

    public static Translation3d primaryForNearZone(RobotState robotState) {
        final double targetX = robotState.getPoopNearTarget().getTargetX();
        // Aim at the amp wall plus some margin in Y
        final double targetY = robotState.getPoopNearTarget().getTargetY();

        return new Translation3d(targetX, targetY, kNominalPoopHeight);
    }

    public static Translation3d generate(RobotState robotState) {
        var fieldToRobot = robotState.getLatestFieldToRobot().getValue();
        double robotX = fieldToRobot.getX();
        double robotY = fieldToRobot.getY();
        Rotation2d robotHeading = fieldToRobot.getRotation();
        if (robotState.isRedAlliance()) {
            // Make robotX BLUE relative.
            robotX = Constants.kFieldLengthMeters - robotX;
            // Make robot heading alliance relative (0 is robot facing away from alliance
            // wall)
            robotHeading = robotHeading.rotateBy(Rotation2d.fromDegrees(180));
        }

        Translation3d target = new Translation3d();
        if (robotX < (kFarWingX + kFarWingPoopBuffer)) {
            target = primaryForNearZone(robotState);

            if ((robotY > 5.5 || robotX < 4.0)
                    && !Util.epsilonEquals(robotHeading.rotateBy(Rotation2d.fromDegrees(180).unaryMinus()).getDegrees(),
                            0, 45)) {
                // When close to the near poop point without stage in the way, shoot line
                // drives.
                target = new Translation3d(target.getX(), target.getY(), kLineDrivePoopHeight);
            }
        } else {
            target = primaryForFarZone();
        }

        // Flip X if red alliance. Y does not flip.
        if (robotState.isRedAlliance()) {
            target = Util.flipRedBlue(target);
        }

        Logger.recordOutput("Poop Pose", target);

        return target;
    }
}
