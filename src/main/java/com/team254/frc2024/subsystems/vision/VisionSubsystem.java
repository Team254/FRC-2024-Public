package com.team254.frc2024.subsystems.vision;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotState;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.MathHelpers;
import com.team254.lib.util.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;

    private final RobotState state;

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private static class PinholeObservation {
        public Translation2d cameraToTag;
        public Pose3d tagPose;
    }

    private double lastProcessedTurretTimestamp = 0.0;
    private double lastProcessedElevatorTimestamp = 0.0;

    public VisionSubsystem(VisionIO io, RobotState state) {
        this.io = io;
        this.state = state;
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        // Read inputs from IO
        io.readInputs(inputs);
        Logger.processInputs("Vision", inputs);

        // Optionally update RobotState
        if (inputs.turretCameraSeesTarget) {
            updateVision(inputs.turretCameraSeesTarget, inputs.turretCameraFiducialObservations,
                    inputs.turretCameraMegatagPoseEstimate, inputs.turretCameraMegatag2PoseEstimate, true);
        } else if (inputs.elevatorCameraSeesTarget) {
            updateVision(inputs.elevatorCameraSeesTarget, inputs.elevatorCameraFiducialObservations,
                    inputs.elevatorCameraMegatagPoseEstimate, inputs.elevatorCameraMegatag2PoseEstimate, false);
        }

        Logger.recordOutput("Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    private void updateVision(boolean cameraSeesTarget, FiducialObservation[] cameraFiducialObservations,
            MegatagPoseEstimate cameraMegatagPoseEstimate, MegatagPoseEstimate cameraMegatag2PoseEstimate,
            boolean isTurretCamera) {
        if (cameraMegatagPoseEstimate != null) {

            String logPreface = "Vision/" + (isTurretCamera ? "Turret/" : "Elevator/");
            var updateTimestamp = cameraMegatagPoseEstimate.timestampSeconds;
            boolean alreadyProcessedTimestamp = (isTurretCamera ? lastProcessedTurretTimestamp
                    : lastProcessedElevatorTimestamp) == updateTimestamp;
            if (!alreadyProcessedTimestamp && cameraSeesTarget) {
                if (!isTurretCamera && !Util.epsilonEquals(state.getElevatorHeight(), 0.0, 0.05))
                    return;
                Optional<VisionFieldPoseEstimate> pinholeEstimate = Optional.empty();// processPinholeVisionEstimate(pinholeObservations,
                                                                                     // updateTimestamp,
                                                                                     // isTurretCamera);

                Optional<VisionFieldPoseEstimate> megatagEstimate = processMegatagPoseEstimate(
                        cameraMegatagPoseEstimate,
                        isTurretCamera);
                Optional<VisionFieldPoseEstimate> megatag2Estimate = processMegatag2PoseEstimate(
                        cameraMegatag2PoseEstimate, isTurretCamera, logPreface);

                boolean used_megatag = false;
                if (megatagEstimate.isPresent()) {
                    if (shouldUseMegatag(cameraMegatagPoseEstimate, cameraFiducialObservations, isTurretCamera,
                            logPreface)) {
                        Logger.recordOutput(logPreface + "MegatagEstimate",
                                megatagEstimate.get().getVisionRobotPoseMeters());
                        state.updateMegatagEstimate(megatagEstimate.get());
                        used_megatag = true;
                    } else {
                        if (megatagEstimate.isPresent()) {
                            Logger.recordOutput(logPreface + "MegatagEstimateRejected",
                                    megatagEstimate.get().getVisionRobotPoseMeters());
                        }
                    }
                }

                if (megatag2Estimate.isPresent() && !used_megatag) {
                    if (shouldUseMegatag2(cameraMegatag2PoseEstimate, isTurretCamera, logPreface)) {
                        Logger.recordOutput(logPreface + "Megatag2Estimate",
                                megatag2Estimate.get().getVisionRobotPoseMeters());
                        state.updateMegatagEstimate(megatag2Estimate.get());
                    } else {
                        if (megatagEstimate.isPresent()) {
                            Logger.recordOutput(logPreface + "Megatag2EstimateRejected",
                                    megatag2Estimate.get().getVisionRobotPoseMeters());
                        }
                    }
                }
                if (pinholeEstimate.isPresent()) {
                    if (shouldUsePinhole(updateTimestamp, isTurretCamera, logPreface)) {
                        Logger.recordOutput(logPreface + "PinholeEstimate",
                                pinholeEstimate.get().getVisionRobotPoseMeters());
                        state.updatePinholeEstimate(pinholeEstimate.get());
                    } else {
                        Logger.recordOutput(logPreface + "PinholeEstimateRejected",
                                pinholeEstimate.get().getVisionRobotPoseMeters());
                    }
                }

                if (isTurretCamera)
                    lastProcessedTurretTimestamp = updateTimestamp;
                else
                    lastProcessedElevatorTimestamp = updateTimestamp;
            }
        }
    }

    @SuppressWarnings("unused")
    private List<PinholeObservation> getPinholeObservations(FiducialObservation[] fiducials, boolean isTurretCamera) {
        // Iterate over the fiducials to make VisionUpdates
        return Arrays.stream(fiducials).map(fiducial -> {
            Optional<Pose3d> tagPoseOptional = Constants.kAprilTagLayout.getTagPose(fiducial.id);
            if (tagPoseOptional.isEmpty()) {
                return null;
            }
            Pose3d tagPose = tagPoseOptional.get();
            Optional<Translation2d> cameraToTarget = getCameraToTargetTranslation(fiducial, tagPose, isTurretCamera);

            if (cameraToTarget.isEmpty()) {
                return null;
            }

            var observation = new PinholeObservation();
            observation.cameraToTag = cameraToTarget.get();
            observation.tagPose = tagPose;
            return observation;
        }).filter(Objects::nonNull).toList();
    }

    private Optional<Translation2d> getCameraToTargetTranslation(FiducialObservation fiducial, Pose3d tagLocation,
            boolean isTurretCamera) {
        // Get the yaw and pitch angles from to target from the camera POV
        double yawRadians = Math.toRadians(fiducial.txnc);
        double pitchRadians = Math.toRadians(fiducial.tync);

        Transform3d cameraToTarget = new Transform3d(new Translation3d(), new Rotation3d(0.0, pitchRadians, 0.0));
        cameraToTarget = cameraToTarget
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, yawRadians)));
        Transform3d cameraGroundPlaneToCamera = new Transform3d(new Translation3d(),
                new Rotation3d(0.0, isTurretCamera ? Constants.kCameraPitchRads : Constants.kCameraBPitchRads, 0));
        Rotation3d cameraGroundPlaneToTarget = new Pose3d().plus(cameraGroundPlaneToCamera.plus(cameraToTarget))
                .getRotation().unaryMinus();

        // Built a unit vector from adjusted rotation.
        // Raw vector: x = 1, y = tan(yaw), z = tan(pitch)
        // Make it a unit vector by dividing each component by magnitude
        // sqrt(x^2+y^2+z^2).
        double tan_ty = Math.tan(cameraGroundPlaneToTarget.getZ()); // y and z switch intentional
        double tan_tz = -Math.tan(cameraGroundPlaneToTarget.getY()); // y and z switch intentional

        if (tan_tz == 0.0) {
            // Protect against divide by zero (i.e. target is at same height as camera).
            return Optional.empty();
        }

        // Find the fixed height difference between the center of the tag and the camera
        // lens
        double differential_height = tagLocation.getZ()
                - (isTurretCamera ? Constants.kCameraHeightOffGroundMeters : Constants.kCameraBHeightOffGroundMeters);

        // We now obtain 3d distance by dividing differential_height by our normalized z
        // component z / (Math.sqrt(x^2+y^2+z^2))
        double distance = differential_height * Math.sqrt(1.0 + tan_tz * tan_tz + tan_ty * tan_ty) / tan_tz;
        // Build a 3d vector from distance (which we now know) and orientation (which we
        // already computed above).
        Translation3d cameraToTargetTranslation = new Translation3d(distance, cameraGroundPlaneToTarget);

        // Grab the x and y components.
        return Optional.of(new Translation2d(cameraToTargetTranslation.getX(), cameraToTargetTranslation.getY()));
    }

    final static Set<Integer> kTagsBlueSpeaker = new HashSet<>(List.of(7, 8));
    final static Set<Integer> kTagsRedSpeaker = new HashSet<>(List.of(3, 4));

    private boolean shouldUseMegatag(MegatagPoseEstimate poseEstimate, FiducialObservation[] fiducials,
            boolean isTurretCamera, String logPreface) {
        final double kMinAreaForTurretMegatagEnabled = 0.4;
        final double kMinAreaForTurretMegatagDisabled = 0.05;

        final double kMinAreaForElevatorMegatagEnabled = 0.4;
        final double kMinAreaForElevatorMegatagDisabled = 0.05;

        double kMinAreaForMegatag = 0.0;
        if (DriverStation.isDisabled()) {
            kMinAreaForMegatag = isTurretCamera ? kMinAreaForTurretMegatagDisabled
                    : kMinAreaForElevatorMegatagDisabled;
        } else {
            kMinAreaForMegatag = isTurretCamera ? kMinAreaForTurretMegatagEnabled
                    : kMinAreaForElevatorMegatagEnabled;
        }

        final int kExpectedTagCount = 2;

        final double kLargeYawThreshold = Units.degreesToRadians(200.0);
        final double kLargeYawEventTimeWindowS = 0.05;

        if (!isTurretCamera) {
            var maxYawVel = state.getMaxAbsDriveYawAngularVelocityInRange(
                    poseEstimate.timestampSeconds - kLargeYawEventTimeWindowS,
                    poseEstimate.timestampSeconds);
            if (maxYawVel.isPresent() && Math.abs(maxYawVel.get()) > kLargeYawThreshold) {
                Logger.recordOutput("Vision/Elevator/MegatagYawAngular", false);
                return false;
            }
            Logger.recordOutput("Vision/Elevator/MegatagYawAngular", true);
        }

        if (poseEstimate.avgTagArea < kMinAreaForMegatag) {
            Logger.recordOutput(logPreface + "megaTagAvgTagArea", false);
            return false;
        }
        Logger.recordOutput(logPreface + "megaTagAvgTagArea", true);

        if (poseEstimate.fiducialIds.length != kExpectedTagCount) {
            Logger.recordOutput(logPreface + "fiducialLength", false);
            return false;
        }
        Logger.recordOutput(logPreface + "fiducialLength", true);

        if (poseEstimate.fiducialIds.length < 1) {
            Logger.recordOutput(logPreface + "fiducialLengthLess1", false);
            return false;
        }
        Logger.recordOutput(logPreface + "fiducialLengthLess1", true);

        if (poseEstimate.fieldToCamera.getTranslation().getNorm() < 1.0) {
            Logger.recordOutput(logPreface + "NormCheck", false);
            return false;
        }
        Logger.recordOutput(logPreface + "NormCheck", true);

        for (var fiducial : fiducials) {
            if (fiducial.ambiguity > .9) {
                Logger.recordOutput(logPreface + "Ambiguity", false);
                return false;
            }
        }
        Logger.recordOutput(logPreface + "Ambiguity", true);

        Set<Integer> seenTags = Arrays.stream(poseEstimate.fiducialIds).boxed()
                .collect(Collectors.toCollection(HashSet::new));
        Set<Integer> expectedTags = state.isRedAlliance() ? kTagsRedSpeaker : kTagsBlueSpeaker;
        var result = expectedTags.equals(seenTags);
        Logger.recordOutput(logPreface + "SeenTags", result);
        return result;
    }

    private boolean shouldUseMegatag2(MegatagPoseEstimate poseEstimate, boolean isTurretCamera, String logPreface) {
        return shouldUsePinhole(poseEstimate.timestampSeconds, isTurretCamera, logPreface);
    }

    private boolean shouldUsePinhole(double timestamp, boolean isTurretCamera, String preface) {
        final double kLargePitchRollYawEventTimeWindowS = 0.1;
        final double kLargePitchRollThreshold = Units.degreesToRadians(10.0);
        final double kLargeYawThreshold = Units.degreesToRadians(100.0);
        if (isTurretCamera) {
            var maxTurretVel = state.getMaxAbsTurretYawAngularVelocityInRange(
                    timestamp - kLargePitchRollYawEventTimeWindowS, timestamp);
            var maxYawVel = state.getMaxAbsDriveYawAngularVelocityInRange(
                    timestamp - kLargePitchRollYawEventTimeWindowS,
                    timestamp);

            if (maxTurretVel.isPresent() && maxYawVel.isPresent() &&
                    Math.abs(maxTurretVel.get() + maxYawVel.get()) > kLargeYawThreshold) {
                Logger.recordOutput(preface + "PinholeTurretAngular", false);
                return false;
            }
            Logger.recordOutput(preface + "PinholeTurretAngular", true);
        } else {
            var maxYawVel = state.getMaxAbsDriveYawAngularVelocityInRange(
                    timestamp - kLargePitchRollYawEventTimeWindowS,
                    timestamp);
            if (maxYawVel.isPresent() && Math.abs(maxYawVel.get()) > kLargeYawThreshold) {
                Logger.recordOutput(preface + "PinholeYawAngular", false);
                return false;
            }
            Logger.recordOutput(preface + "PinholeYawAngular", true);
        }

        var maxPitchVel = state.getMaxAbsDrivePitchAngularVelocityInRange(
                timestamp - kLargePitchRollYawEventTimeWindowS,
                timestamp);
        if (maxPitchVel.isPresent() && Math.abs(maxPitchVel.get()) > kLargePitchRollThreshold) {
            Logger.recordOutput(preface + "PinholePitchAngular", false);
            return false;
        }
        Logger.recordOutput(preface + "PinholePitchAngular", true);

        var maxRollVel = state.getMaxAbsDriveRollAngularVelocityInRange(timestamp - kLargePitchRollYawEventTimeWindowS,
                timestamp);
        if (maxRollVel.isPresent() && Math.abs(maxRollVel.get()) > kLargePitchRollThreshold) {
            Logger.recordOutput(preface + "PinholeRollAngular", false);
            return false;
        }
        Logger.recordOutput(preface + "PinholeRollAngular", true);

        return true;
    }

    private Optional<Pose2d> getFieldToRobotEstimate(MegatagPoseEstimate poseEstimate, boolean isTurretCamera) {
        var fieldToCamera = poseEstimate.fieldToCamera;
        if (fieldToCamera.getX() == 0.0) {
            return Optional.empty();
        }
        var turretToCameraTransform = state.getTurretToCamera(isTurretCamera);
        var cameraToTurretTransform = turretToCameraTransform.inverse();
        var fieldToTurretPose = fieldToCamera.plus(cameraToTurretTransform);
        var fieldToRobotEstimate = MathHelpers.kPose2dZero;
        if (isTurretCamera) {
            var robotToTurretObservation = state.getRobotToTurret(poseEstimate.timestampSeconds);
            if (robotToTurretObservation.isEmpty()) {
                return Optional.empty();
            }
            var turretToRobot = MathHelpers.transform2dFromRotation(robotToTurretObservation.get().unaryMinus());
            fieldToRobotEstimate = fieldToTurretPose.plus(turretToRobot);
        } else {
            fieldToRobotEstimate = fieldToCamera.plus(turretToCameraTransform.inverse());
        }

        return Optional.of(fieldToRobotEstimate);
    }

    private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(MegatagPoseEstimate poseEstimate,
            boolean isTurretCamera, String logPreface) {
        var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
        if (loggedFieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera);
        if (maybeFieldToRobotEstimate.isEmpty())
            return Optional.empty();
        var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

        // distance from current pose to vision estimated pose
        double poseDifference = fieldToRobotEstimate.getTranslation()
                .getDistance(loggedFieldToRobot.get().getTranslation());

        var defaultSet = state.isRedAlliance() ? kTagsRedSpeaker : kTagsBlueSpeaker;
        Set<Integer> speakerTags = new HashSet<>(defaultSet);
        speakerTags.removeAll(
                Arrays.stream(poseEstimate.fiducialIds).boxed().collect(Collectors.toCollection(HashSet::new)));
        boolean seesSpeakerTags = speakerTags.size() < 2;

        double xyStds;
        if (poseEstimate.fiducialIds.length > 0) {
            // multiple targets detected
            if (poseEstimate.fiducialIds.length >= 2 && poseEstimate.avgTagArea > 0.1) {
                xyStds = 0.2;
            }
            // we detect at least one of our speaker tags and we're close to it.
            else if (seesSpeakerTags && poseEstimate.avgTagArea > 0.2) {
                xyStds = 0.5;
            }
            // 1 target with large area and close to estimated pose
            else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 0.5;
            }
            // 1 target farther away and estimated pose is close
            else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 1.0;
            } else if (poseEstimate.fiducialIds.length > 1) {
                xyStds = 1.2;
            } else {
                xyStds = 2.0;
            }

            Logger.recordOutput(logPreface + "Megatag2StdDev", xyStds);
            Logger.recordOutput(logPreface + "Megatag2AvgTagArea", poseEstimate.avgTagArea);
            Logger.recordOutput(logPreface + "Megatag2PoseDifference", poseDifference);

            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(50.0));
            fieldToRobotEstimate = new Pose2d(fieldToRobotEstimate.getTranslation(),
                    loggedFieldToRobot.get().getRotation());
            return Optional.of(
                    new VisionFieldPoseEstimate(fieldToRobotEstimate, poseEstimate.timestampSeconds,
                            visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(MegatagPoseEstimate poseEstimate,
            boolean isTurretCamera) {
        var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
        if (loggedFieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera);
        if (maybeFieldToRobotEstimate.isEmpty())
            return Optional.empty();
        var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

        // distance from current pose to vision estimated pose
        double poseDifference = fieldToRobotEstimate.getTranslation()
                .getDistance(loggedFieldToRobot.get().getTranslation());

        if (poseEstimate.fiducialIds.length > 0) {
            double xyStds = 1.0;
            double degStds = 12;
            // multiple targets detected
            if (poseEstimate.fiducialIds.length >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }

            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
            return Optional.of(
                    new VisionFieldPoseEstimate(fieldToRobotEstimate, poseEstimate.timestampSeconds,
                            visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    @SuppressWarnings("unused")
    private Optional<VisionFieldPoseEstimate> processPinholeVisionEstimate(List<PinholeObservation> observations,
            double timestamp, boolean isTurretCamera) {
        observations = observations.stream().filter((observation) -> {
            if (observation.cameraToTag.getNorm() < 10.0) {
                return true;
            }
            Logger.recordOutput("Vision/RejectOnNormTimestamp", RobotTime.getTimestampSeconds());
            return false;
        }).toList();

        if (observations.isEmpty())
            return Optional.empty();

        int num_updates = 0;
        double x = 0.0, y = 0.0;
        // All timestamps and rotations are the same. If that changes, need to revisit.
        Rotation2d rotation = MathHelpers.kRotation2dZero;
        double avgRange = 0.0;
        var poseTurret = state.getRobotToTurret(timestamp);
        var poseRobot = state.getFieldToRobot(timestamp);
        if (poseRobot.isEmpty() || poseTurret.isEmpty()) {
            return Optional.empty();
        }
        for (var observation : observations) {
            Pose2d fieldToRobotEstimate = estimateFieldToRobot(
                    observation.cameraToTag,
                    observation.tagPose,
                    poseTurret.get(),
                    poseRobot.get().getRotation(),
                    MathHelpers.kRotation2dZero, isTurretCamera);
            x += fieldToRobotEstimate.getX();
            y += fieldToRobotEstimate.getY();
            rotation = fieldToRobotEstimate.getRotation();
            avgRange += observation.cameraToTag.getNorm();
            num_updates++;
        }

        if (num_updates == 0)
            return Optional.empty();

        avgRange /= num_updates;

        double xyStds = 100.0;
        var fieldToRobotEstimate = new Pose2d(x / num_updates, y / num_updates, rotation);
        var poseDifference = fieldToRobotEstimate.getTranslation().getDistance(poseRobot.get().getTranslation());
        // multiple targets detected
        if (observations.size() >= 2 && avgRange < 3.0) {
            xyStds = 0.2;
        } else if (avgRange < 5.0 && poseDifference < 0.5) {
            xyStds = 0.5;
        }
        // 1 target farther away and estimated pose is close
        else if (avgRange < 3.0 && poseDifference < 0.3) {
            xyStds = 1.0;
        } else if (observations.size() > 1) {
            xyStds = 1.2;
        } else {
            xyStds = 2.0;
        }
        ;

        final double rotStdDev = Units.degreesToRadians(50);
        Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, rotStdDev);
        return Optional.of(new VisionFieldPoseEstimate(fieldToRobotEstimate, timestamp, visionMeasurementStdDevs));
    }

    private Pose2d estimateFieldToRobot(Translation2d cameraToTarget, Pose3d fieldToTarget, Rotation2d robotToTurret,
            Rotation2d gyroAngle, Rotation2d cameraYawOffset, boolean isTurretCamera) {
        Transform2d cameraToTargetFixed = MathHelpers
                .transform2dFromTranslation(cameraToTarget.rotateBy(cameraYawOffset));
        Transform2d turretToTarget = state.getTurretToCamera(isTurretCamera).plus(cameraToTargetFixed);
        // In robot frame
        Transform2d robotToTarget = turretToTarget;
        if (isTurretCamera) {
            robotToTarget = MathHelpers.transform2dFromRotation(robotToTurret).plus(turretToTarget);
        }

        // In field frame
        Transform2d robotToTargetField = MathHelpers
                .transform2dFromTranslation(robotToTarget.getTranslation().rotateBy(gyroAngle));

        // In field frame
        Pose2d fieldToTarget2d = MathHelpers.pose2dFromTranslation(fieldToTarget.toPose2d().getTranslation());

        Pose2d fieldToRobot = fieldToTarget2d.transformBy(MathHelpers.transform2dFromTranslation(
                robotToTargetField.getTranslation().unaryMinus()));

        Pose2d fieldToRobotYawAdjusted = new Pose2d(fieldToRobot.getTranslation(), gyroAngle);
        return fieldToRobotYawAdjusted;
    }

}
