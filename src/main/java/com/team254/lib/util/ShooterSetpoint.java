package com.team254.lib.util;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;
import java.util.function.Supplier;

public class ShooterSetpoint {

    public static Optional<Double> overrideRPS = Optional.empty();

    private double shooterRPS;
    private double shooterStage1RPS = 14.4;
    private double turretRadiansFromCenter;
    private double turretFF;
    private double hoodRadians;
    private double hoodFF;
    private boolean isValid;

    public ShooterSetpoint(double shooterRPS, double turretRadiansFromCenter, double turretFF, double hoodRadians,
            double hoodFF, boolean isValid) {
        this.shooterRPS = shooterRPS;
        this.turretRadiansFromCenter = turretRadiansFromCenter;
        this.turretFF = turretFF;
        this.hoodRadians = hoodRadians;
        this.hoodFF = hoodFF;
        this.isValid = isValid;
    }

    public ShooterSetpoint(double shooterRPS, double turretRadiansFromCenter, double turretFF, double hoodRadians,
            double hoodFF) {
        this.shooterRPS = shooterRPS;
        this.turretRadiansFromCenter = turretRadiansFromCenter;
        this.turretFF = turretFF;
        this.hoodRadians = hoodRadians;
        this.hoodFF = hoodFF;
        this.isValid = true;
    }

    public static void clearOverrideRPS() {
        overrideRPS = Optional.empty();
    }

    public static void setOverrideRPS(double rps) {
        overrideRPS = Optional.of(rps);
    }

    public boolean getIsValid() {
        return this.isValid;
    }

    private static ShooterSetpoint makeSetpoint(RobotState robotState, Rotation2d robotToTargetRotation,
            Translation3d robotToTargetTranslation,
            double pitchAngleRads, double launchSpeedMetersPerSec) {
        // turret
        Rotation2d turretRotationRobotFrame = robotToTargetRotation
                .minus(robotState.getLatestFieldToRobot().getValue().getRotation());
        Rotation2d turretRotationTurretFrame = turretRotationRobotFrame
                .rotateBy(MathHelpers.kRotation2dPi).rotateBy(Constants.ShooterConstants.kTurretToShotCorrection);

        // hood
        var hoodZeroedAngle = Rotation2d.fromDegrees(Constants.HoodConstants.kHoodZeroedAngleDegrees);
        double hoodAngle = hoodZeroedAngle.getRadians() - pitchAngleRads;

        // Feedfowards
        var robotSpeeds = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();

        var robotToTargetXY = new Translation2d(robotToTargetTranslation.getX(), robotToTargetTranslation.getY());

        // In this frame, x = radial component (positive towards goal)
        // y = tangential component (positive means turret needs negative lead)
        var targetFrameToRobot = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                .rotateBy(
                        robotToTargetXY.getAngle());

        var tangent = targetFrameToRobot.getY();
        var angular = robotSpeeds.omegaRadiansPerSecond;
        var distanceToTarget = robotToTargetXY.getNorm();
        var turretFF = -(angular + tangent / distanceToTarget);
        // This is the deriative of atan2 accounting for the frame that the hood is
        // defined in.
        var hoodFF = targetFrameToRobot.getX() * -robotToTargetTranslation.getZ() /
                (distanceToTarget * distanceToTarget +
                        robotToTargetTranslation.getZ() * robotToTargetTranslation.getZ());

        boolean validSetpont = true;
        double shooterRPS = launchSpeedMetersPerSec / Constants.ShooterConstants.kRingLaunchVelMetersPerSecPerRotPerSec;
        if (shooterRPS > Constants.ShooterConstants.kShooterStage2RPSCap) {
            shooterRPS = Constants.ShooterConstants.kShooterStage2RPSCap;
            validSetpont = false;
        }

        return new ShooterSetpoint(shooterRPS,
                turretRotationTurretFrame.getRadians(),
                turretFF,
                hoodAngle,
                hoodFF, validSetpont);
    }

    public static Supplier<ShooterSetpoint> poopSetpointSupplier(RobotState robotState) {
        return poopSetpointSupplier(() -> PoopTargetFactory.generate(robotState), robotState);
    }

    public static Supplier<ShooterSetpoint> poopSetpointSupplier(Supplier<Translation3d> targetPoint,
            RobotState robotState) {
        return Util.memoizeByIteration(robotState.getIterationSupplier(),
                () -> fromPoopPose(targetPoint.get(), robotState));
    }

    private static ShooterSetpoint fromPoopPose(Translation3d target, RobotState robotState) {
        double maxPoopHeight = target.getZ();
        target = new Translation3d(target.getX(), target.getY(), 0.0);
        var fieldToRobot = robotState.getLatestFieldToRobot();
        var vRobot = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
        Translation3d d = target.minus(
                new Translation3d(fieldToRobot.getValue().getX(), fieldToRobot.getValue().getY(),
                        Constants.kNoteReleaseHeight));
        var hoodZeroedAngle = Rotation2d.fromDegrees(Constants.HoodConstants.kHoodZeroedAngleDegrees);

        double apexHeight = maxPoopHeight;
        double pitchAngleRads = 0.0;
        double launchSpeedMetersPerSec = 0.0;
        Rotation2d robotToTargetRotation = MathHelpers.kRotation2dZero;

        final int max_num_iterations = 10;
        double minApexHeight = 0.0;
        double maxApexHeight = apexHeight;
        for (int i = 0; i < max_num_iterations; ++i) {
            // Try to aim at our nominal apex height, then reduce it if we need to.
            final double kG = -9.81;
            double vz = Math.sqrt(-2.0 * kG * (apexHeight - Constants.kNoteReleaseHeight));
            double t_apex = vz / -kG;
            double t_fall = Math.sqrt(2.0 * apexHeight / -kG);
            double t_total = t_apex + t_fall;
            double vx = (d.getX() - vRobot.vxMetersPerSecond * t_total) / t_total;
            double vy = (d.getY() - vRobot.vyMetersPerSecond * t_total) / t_total;

            double shotXY = Math.sqrt(vx * vx + vy * vy);
            pitchAngleRads = Math.atan2(vz, shotXY);

            double hoodAngle = hoodZeroedAngle.getRadians() - pitchAngleRads;

            // Hood needs to go too far vertical, so we need to reduce apex height.
            // Solving exactly for the extremal hood position yields a quartic function, so
            // just binary search over
            // apex heights to find something close.
            if (hoodAngle < Constants.HoodConstants.kHoodMinPositionRadians) {
                // We have to aim lower. Don't remember the launch parameters because this angle
                // is infeasible.
                maxApexHeight = Math.min(apexHeight, maxApexHeight);
                apexHeight = (maxApexHeight - minApexHeight) / 2.0 + minApexHeight;
            } else if (apexHeight < Constants.ShooterConstants.kPoopMaxApexHeight) {
                // We can aim higher. Remember the parameters in case this is the best we find.
                launchSpeedMetersPerSec = Math.sqrt(vz * vz + shotXY * shotXY);
                robotToTargetRotation = new Rotation2d(vx, vy);
                minApexHeight = Math.max(apexHeight, minApexHeight);
                apexHeight = (maxApexHeight - minApexHeight) / 2.0 + minApexHeight;
            } else {
                // Found an exact solution that achieves our nominal apex height.
                launchSpeedMetersPerSec = Math.sqrt(vz * vz + shotXY * shotXY);
                robotToTargetRotation = new Rotation2d(vx, vy);
                break;
            }
        }
        return makeSetpoint(robotState, robotToTargetRotation, d, pitchAngleRads, launchSpeedMetersPerSec);
    }

    public static Supplier<ShooterSetpoint> autoSetpointSupplier(RobotState robotState) {
        return autoSetpointSupplier(() -> SpeakerTargetFactory.generate(robotState), robotState);
    }

    public static Supplier<ShooterSetpoint> autoSetpointSupplier(Supplier<Translation3d> speakerTargetPoint,
            RobotState robotState) {
        return Util.memoizeByIteration(robotState.getIterationSupplier(),
                () -> fromAutoTarget(speakerTargetPoint.get(), robotState));
    }

    public static Supplier<ShooterSetpoint> speakerSetpointSupplier(RobotState robotState) {
        return speakerSetpointSupplier(() -> SpeakerTargetFactory.generate(robotState), robotState);
    }

    public static Supplier<ShooterSetpoint> speakerSetpointSupplier(Supplier<Translation3d> targetPoint,
            RobotState robotState) {
        return Util.memoizeByIteration(robotState.getIterationSupplier(),
                () -> fromSpeakerTarget(targetPoint.get(), robotState));
    }

    private static ShooterSetpoint fromAutoTarget(Translation3d speakerTarget, RobotState robotState) {
        var setpoint = fromSpeakerTarget(speakerTarget, robotState);
        if (Timer.getFPGATimestamp() - robotState.getAutoStartTime() <= 2) {
            setpoint.shooterRPS = Constants.ShooterConstants.kPreloadShotRPS;
        }
        return setpoint;
    }

    private static ShooterSetpoint fromSpeakerTarget(Translation3d target, RobotState robotState) {
        // turret
        Pose2d fieldToRobot;
        final boolean kUsePrediction = true;
        final double kPredictionLookaheadTime = 0.05;
        if (kUsePrediction) {
            fieldToRobot = robotState.getPredictedFieldToRobot(kPredictionLookaheadTime);
        } else {
            fieldToRobot = robotState.getLatestFieldToRobot().getValue();
        }
        Rotation2d robotToTargetRotation;
        Translation3d robotToTargetTranslation;
        double pitchAngleRads;

        var distanceToTarget = new Translation2d(
                target.getX() - fieldToRobot.getX(),
                target.getY() - fieldToRobot.getY()).getNorm();

        double launchSpeedRPS = 0.0;
        if (distanceToTarget < Constants.ShooterConstants.kShooterStage2MaxShortRangeDistance) {
            launchSpeedRPS = Constants.ShooterConstants.kShooterStage2RPSShortRange;
        } else if (distanceToTarget > Constants.ShooterConstants.kShooterStage2MinLongRangeDistance) {
            launchSpeedRPS = Constants.ShooterConstants.kShooterStage2RPSLongRange;
        } else {
            var x = (distanceToTarget - Constants.ShooterConstants.kShooterStage2MaxShortRangeDistance) /
                    (Constants.ShooterConstants.kShooterStage2MinLongRangeDistance
                            - Constants.ShooterConstants.kShooterStage2MaxShortRangeDistance);
            launchSpeedRPS = Util.interpolate(Constants.ShooterConstants.kShooterStage2RPSShortRange,
                    Constants.ShooterConstants.kShooterStage2RPSLongRange, x);
        }

        // if (overrideRPS.isPresent()) {
        // launchSpeedRPS = overrideRPS.get();
        // }

        double launchSpeedMetersPerSec = Constants.ShooterConstants.kRingLaunchVelMetersPerSecPerRotPerSec *
                launchSpeedRPS;

        boolean kUseMotionCompensation = true;
        if (kUseMotionCompensation) {
            var vRobot = robotState.getLatestMeasuredFieldRelativeChassisSpeeds();
            var vShot = launchSpeedMetersPerSec;

            // Solve quadratic equation to obtain time of flight of ring.
            // a = vx^2+vy^2-shot_vel^2
            // b = -2*((tx-rx)*vx+(ty-ry)*vy)
            // c = (tx-rx)^2+(ty-ry)^2+dz^2
            var a = vRobot.vxMetersPerSecond * vRobot.vxMetersPerSecond +
                    vRobot.vyMetersPerSecond * vRobot.vyMetersPerSecond -
                    vShot * vShot;
            if (Math.abs(a) < Util.kEpsilon) {
                // Not a quadratic equation, cheat a little bit to make it one.
                vShot = 1.01 * vShot;
            }
            Translation3d d = target.minus(
                    new Translation3d(fieldToRobot.getX(), fieldToRobot.getY(), Constants.kNoteReleaseHeight));
            var b = -2.0 * (d.getX() * vRobot.vxMetersPerSecond +
                    d.getY() * vRobot.vyMetersPerSecond);
            var c = d.getX() * d.getX() + d.getY() * d.getY() + d.getZ() * d.getZ();

            var discriminant = b * b - 4.0 * a * c;
            if (discriminant < 0.0) {
                discriminant = 0.0;
            }
            var t = (-b - Math.sqrt(discriminant)) / (2.0 * a);
            var shot = new Translation3d((d.getX() - vRobot.vxMetersPerSecond * t) / t,
                    (d.getY() - vRobot.vyMetersPerSecond * t) / t,
                    (d.getZ() / t));
            robotToTargetRotation = new Rotation2d(shot.getX(), shot.getY());
            var xyVel = Math.sqrt(shot.getX() * shot.getX() + shot.getY() * shot.getY());
            pitchAngleRads = Math.atan2(shot.getZ(), xyVel);

            boolean kUseGravityCompensation = true;
            final double kG = -9.81;
            if (kUseGravityCompensation) {
                boolean kUseLiftCompensation = true;
                var drop = 0.5 * t * t * kG;
                if (kUseLiftCompensation) {
                    // But, v here is (distance / t), so v^2 * t^2 just becomes distance^2, which is
                    // c.
                    // That's neat, huh.
                    drop += 0.5 * Constants.ShooterConstants.kRingLaunchLiftCoeff * c;
                }
                pitchAngleRads = Math.atan2((d.getZ() - drop) / t, xyVel);
                vShot = Math.sqrt((d.getZ() - drop) * (d.getZ() - drop) / (t * t) + xyVel * xyVel);
            }
            launchSpeedMetersPerSec = vShot;
            robotToTargetTranslation = d;
        } else {
            robotToTargetRotation = new Rotation2d(
                    target.getX() - fieldToRobot.getX(),
                    target.getY() - fieldToRobot.getY());
            var differential_height = target.getZ() - Constants.kNoteReleaseHeight;
            pitchAngleRads = Math.atan2(differential_height, distanceToTarget);
            robotToTargetTranslation = new Translation3d(
                    target.getX() - fieldToRobot.getX(),
                    target.getY() - fieldToRobot.getY(), differential_height);
        }
        return makeSetpoint(robotState, robotToTargetRotation, robotToTargetTranslation, pitchAngleRads,
                launchSpeedMetersPerSec);
    }

    public double getShooterRPS() {
        return shooterRPS;
    }

    public double getShooterStage1RPS() {
        return shooterStage1RPS;
    }

    public double getTurretRadiansFromCenter() {
        return turretRadiansFromCenter;
    }

    public double getTurretFF() {
        return turretFF;
    }

    public double getHoodRadians() {
        return hoodRadians;
    }

    public double getHoodFF() {
        return hoodFF;
    }
}
