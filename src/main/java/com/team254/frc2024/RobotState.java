package com.team254.frc2024;

import com.team254.frc2024.controlboard.ModalControls;
import com.team254.frc2024.subsystems.vision.VisionFieldPoseEstimate;
import com.team254.lib.time.RobotTime;
import com.team254.lib.util.ConcurrentTimeInterpolatableBuffer;
import com.team254.lib.util.MathHelpers;
import com.team254.lib.util.PoopTargetFactory.NearTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

public class RobotState {

    public final static double LOOKBACK_TIME = 1.0;

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        // Make sure to add one sample to these methods to protect callers against null.
        fieldToRobot.addSample(0.0, MathHelpers.kPose2dZero);
        robotToTurret.addSample(0.0, MathHelpers.kRotation2dZero);
        turretAngularVelocity.addSample(0.0, 0.0);
        driveYawAngularVelocity.addSample(0.0, 0.0);
        turretPositionRadians.addSample(0.0, 0.0);
    }

    // State of robot.

    // Kinematic Frames
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer
            .createBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Rotation2d> robotToTurret = ConcurrentTimeInterpolatableBuffer
            .createBuffer(LOOKBACK_TIME);
    private static final Transform2d TURRET_TO_CAMERA = new Transform2d(Constants.kTurretToCameraX,
            Constants.kTurretToCameraY,
            MathHelpers.kRotation2dZero);
    private static final Transform2d ROBOT_TO_CAMERA_B = new Transform2d(Constants.kTurretToCameraBX,
            Constants.kTurretToCameraBY,
            MathHelpers.kRotation2dZero);
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());

    private final AtomicInteger iteration = new AtomicInteger(0);

    private double lastUsedMegatagTimestamp = 0;
    private double lastTriggeredIntakeSensorTimestamp = 0;
    private ConcurrentTimeInterpolatableBuffer<Double> turretAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> turretPositionRadians = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelX = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelY = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

    private double autoStartTime;

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel.set(true);
    }

    public void disablePathCancel() {
        enablePathCancel.set(false);
    }

    public boolean getPathCancel() {
        return enablePathCancel.get();
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void incrementIterationCount() {
        iteration.incrementAndGet();
    }

    public int getIteration() {
        return iteration.get();
    }

    public IntSupplier getIterationSupplier() {
        return () -> getIteration();
    }

    public void addDriveMotionMeasurements(double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelY.addSample(timestamp, accelY);
        this.accelX.addSample(timestamp, accelX);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot
                .exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
    }

    public Pose2d getLatestFieldToRobotCenter() {
        return fieldToRobot.getLatest().getValue().transformBy(Constants.kTurretToRobotCenter);
    }

    // This has rotation and radians to allow for wrapping tracking.
    public void addTurretUpdates(double timestamp,
            Rotation2d turretRotation,
            double turretRadians,
            double angularYawRadsPerS) {
        // turret frame 180 degrees off from robot frame
        robotToTurret.addSample(timestamp, turretRotation.rotateBy(MathHelpers.kRotation2dPi));
        this.turretAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.turretPositionRadians.addSample(timestamp, turretRadians);
    }

    public double getLatestTurretPositionRadians() {
        return this.turretPositionRadians.getInternalBuffer().lastEntry().getValue();
    }

    public double getLatestTurretAngularVelocity() {
        return this.turretAngularVelocity.getInternalBuffer().lastEntry().getValue();
    }

    private Rotation2d hoodRotation = new Rotation2d();

    public void addHoodRotation(Rotation2d rotationFromZero) {
        hoodRotation = rotationFromZero;
    }

    public Rotation2d getHoodRotation() {
        return hoodRotation;
    }

    private double elevatorHeightM = 0.0;

    public void setElevatorHeight(double heightM) {
        this.elevatorHeightM = heightM;
    }

    public double getElevatorHeight() {
        return elevatorHeightM;
    }

    private double climberRotations = 0.0;

    public void setClimberRotations(double rotations) {
        this.climberRotations = rotations;
    }

    private NearTarget poopNearTarget = NearTarget.DEEP_AMP;

    public void setPoopNearTarget(NearTarget nearTarget) {
        poopNearTarget = nearTarget;
    }

    public NearTarget getPoopNearTarget() {
        return poopNearTarget;
    }

    public Optional<Rotation2d> getRobotToTurret(double timestamp) {
        return robotToTurret.getSample(timestamp);
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public Transform2d getTurretToCamera(boolean isTurretCamera) {
        return isTurretCamera ? TURRET_TO_CAMERA : ROBOT_TO_CAMERA_B;
    }

    public Map.Entry<Double, Rotation2d> getLatestRobotToTurret() {
        return robotToTurret.getLatest();
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    public Optional<Double> getTurretAngularVelocity(double timestamp) {
        return turretAngularVelocity.getSample(timestamp);
    }

    private Optional<Double> getMaxAbsValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime,
            double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty())
            return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get()))
            return max;
        else
            return min;
    }

    public Optional<Double> getMaxAbsTurretYawAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(turretAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        // Gyro yaw rate not set in sim.
        if (Robot.isReal())
            return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = Timer.getFPGATimestamp();
        visionEstimateConsumer.accept(megatagEstimate);
    }

    public void updatePinholeEstimate(VisionFieldPoseEstimate pinholeEstimate) {
        visionEstimateConsumer.accept(pinholeEstimate);
    }

    public void updateLastTriggeredIntakeSensorTimestamp(boolean triggered) {
        if (triggered)
            lastTriggeredIntakeSensorTimestamp = RobotTime.getTimestampSeconds();
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public double lastTriggeredIntakeSensorTimestamp() {
        return lastTriggeredIntakeSensorTimestamp;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public void updateLogger() {
        if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/YawAngularVelocity",
                    this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollAngularVelocity",
                    this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchAngularVelocity",
                    this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelX", this.accelX.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelY", this.accelY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput("RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeed());
    }

    Pose3d ampPose3d = new Pose3d();
    Pose3d climberPose3d = new Pose3d();
    Pose3d hoodPose3d = new Pose3d();
    Pose3d shooterPose3d = new Pose3d();
    Pose3d turretPose3d = new Pose3d();

    public void updateViz() {
        if (getLatestRobotToTurret().getValue() != null) {
            shooterPose3d = new Pose3d(new Translation3d(),
                    new Rotation3d(0, 0,
                            getLatestRobotToTurret().getValue().getRadians()));
            hoodPose3d = new Pose3d(new Translation3d(),
                    new Rotation3d(0, 0,
                            getLatestRobotToTurret().getValue().getRadians()));
        }
        ampPose3d = new Pose3d(
                new Translation3d(0.0, 0.0, this.elevatorHeightM),
                new Rotation3d());
        var climberRot = MathUtil.interpolate(0.0, -Math.PI / 2.0,
                this.climberRotations /
                        (Constants.ClimberConstants.kForwardMaxPositionRotations
                                * Constants.kClimberConfig.unitToRotorRatio));
        climberPose3d = new Pose3d(
                new Translation3d(0, 0.0, 0.15), new Rotation3d(0.0, climberRot, 0.0));
        // model_0 is elevator
        // model_1 is climber
        // model_2 is hood plates
        // model_3 is shooter
        // model_4 is turret
        Logger.recordOutput("ComponentsPoseArray",
                new Pose3d[] { ampPose3d, climberPose3d, hoodPose3d, shooterPose3d, turretPose3d });
    }

    public void logControllerMode() {
        Logger.recordOutput("Controller Mode", ModalControls.getInstance().getMode().toString());
    }
}
