package com.team254.frc2024.subsystems.vision;

import com.team254.frc2024.Constants;
import com.team254.frc2024.RobotState;
import com.team254.frc2024.simulation.SimulatedRobotState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Simulate the vision system.
// Please see the following link for example code
// https://github.com/PhotonVision/photonvision/blob/2a6fa1b6ac81f239c59d724da5339f608897c510/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
public class VisionIOSimPhoton extends VisionIOHardwareLimelight {
    private final PhotonCamera turretCamera = new PhotonCamera("turretCamera");
    private final PhotonCamera elevatorCamera = new PhotonCamera("elevatorCamera");
    private final PhotonCamera elevatorNoteCamera = new PhotonCamera("elevatorNoteCamera");
    PhotonCameraSim turretCameraSim;
    PhotonCameraSim elevatorCameraSim;
    PhotonCameraSim elevatorNoteCameraSim;
    private final VisionSystemSim visionSim;
    private final VisionSystemSim objectSim;

    private final RobotState state;

    private final SimulatedRobotState simRobotState;

    private final int kResWidth = 1280;
    private final int kResHeight = 800;

    public VisionIOSimPhoton(RobotState state, SimulatedRobotState simRobotState) {
        super(state);
        this.state = state;
        this.simRobotState = simRobotState;

        // Create the vision system simulation which handles cameras and targets on the
        // field.
        visionSim = new VisionSystemSim("main");
        objectSim = new VisionSystemSim("object");

        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        visionSim.addAprilTags(Constants.kAprilTagLayout);

        // Create simulated camera properties. These can be set to mimic your actual
        // camera.
        var turretProp = new SimCameraProperties();
        turretProp.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
        turretProp.setCalibError(0.35, 0.10);
        turretProp.setFPS(15);
        turretProp.setAvgLatencyMs(20);
        turretProp.setLatencyStdDevMs(5);

        var elevatorProp = new SimCameraProperties();
        elevatorProp = turretProp.copy();

        // Create a PhotonCameraSim which will update the linked PhotonCamera's values
        // with visible
        // targets.
        // Instance variables
        turretCameraSim = new PhotonCameraSim(turretCamera, turretProp);
        elevatorCameraSim = new PhotonCameraSim(elevatorCamera, elevatorProp);
        elevatorNoteCameraSim = new PhotonCameraSim(elevatorNoteCamera, elevatorProp);

        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(turretCameraSim, new Transform3d());

        Transform3d robotToElevatorCamera = new Transform3d(
                new Translation3d(
                        Constants.kTurretToCameraBX,
                        Constants.kTurretToCameraBY,
                        Constants.kCameraBHeightOffGroundMeters),
                new Rotation3d(0.0, -Constants.kCameraBPitchRads, 0.0));
        visionSim.addCamera(elevatorCameraSim, robotToElevatorCamera);
        objectSim.addCamera(elevatorNoteCameraSim, robotToElevatorCamera);

        elevatorNoteCameraSim.enableRawStream(true);
        elevatorNoteCameraSim.enableProcessedStream(true);
        elevatorNoteCameraSim.enableDrawWireframe(true);

        // Enable the raw and processed streams. (http://localhost:1181 / 1182)
        turretCameraSim.enableRawStream(true);
        turretCameraSim.enableProcessedStream(true);
        elevatorCameraSim.enableRawStream(true);
        elevatorCameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        turretCameraSim.enableDrawWireframe(true);
        elevatorCameraSim.enableDrawWireframe(true);

        for (var entry : Constants.kMidlineNoteTranslations.entrySet()) {
            var translation = entry.getValue();
            addNoteTarget(new Pose3d(translation.getX(),
                    translation.getY(), Units.inchesToMeters(1.0),
                    new Rotation3d()));
        }
    }

    public void addNoteTarget(Pose3d pose) {
        TargetModel noteModel = new TargetModel(
                Units.inchesToMeters(14.0), Units.inchesToMeters(14.0),
                Units.inchesToMeters(2.0));
        VisionTargetSim visionTarget = new VisionTargetSim(pose, noteModel);
        objectSim.addVisionTargets(visionTarget);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        // Abuse the readInputs periodic call to update the sim

        // Move the vision sim robot on the field
        Pose2d estimatedPose = simRobotState.getLatestFieldToRobot();
        if (estimatedPose != null) {
            visionSim.update(estimatedPose);
            objectSim.update(estimatedPose);
            Logger.recordOutput("Vision/SimIO/updateSimPose", estimatedPose);
        }

        var turretRotation = state.getLatestRobotToTurret();
        if (turretRotation != null) {
            Transform3d robotToTurret = new Transform3d(
                    new Translation3d(),
                    new Rotation3d(0.0, 0.0, turretRotation.getValue().getRadians()));
            Transform3d turretToCamera = new Transform3d(
                    new Translation3d(Constants.kTurretToCameraX,
                            Constants.kTurretToCameraY,
                            Constants.kCameraHeightOffGroundMeters),
                    new Rotation3d(0.0, -Constants.kCameraPitchRads, 0.0));
            Transform3d robotToCamera = robotToTurret.plus(turretToCamera);
            visionSim.adjustCamera(turretCameraSim, robotToCamera);
        }
        NetworkTable turretTable = NetworkTableInstance.getDefault().getTable(Constants.kLimelightTableName);
        NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable(Constants.kLimelightBTableName);
        // Write to limelight table
        writeToTable(turretCamera.getLatestResult(), turretTable);
        writeToTable(elevatorCamera.getLatestResult(), elevatorTable);

        super.readInputs(inputs);
    }

    private void writeToTable(PhotonPipelineResult result, NetworkTable table) {
        // Write to limelight table
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            Transform3d best = result.getMultiTagResult().estimatedPose.best;
            Pose2d fieldToCamera = new Pose2d(best.getTranslation().toTranslation2d(),
                    best.getRotation().toRotation2d());
            List<Double> pose_data = new ArrayList<>(Arrays.asList(
                    best.getX(), // 0: X
                    best.getY(), // 1: Y
                    best.getZ(), // 2: Z,
                    0.0, // 3: roll
                    0.0, // 4: pitch
                    fieldToCamera.getRotation().getDegrees(), // 5: yaw
                    result.getLatencyMillis(), // 6: latency ms,
                    (double) result.getMultiTagResult().fiducialIDsUsed.size(), // 7: tag count
                    0.0, // 8: tag span
                    0.0, // 9: tag dist
                    result.getBestTarget().getArea() // 10: tag area
            ));
            // Add RawFiducials
            // This is super inefficient but it's sim only, who cares.
            for (var target : result.targets) {
                pose_data.add((double) target.getFiducialId()); // 0: id
                pose_data.add(target.getYaw()); // 1: txnc
                pose_data.add(target.getPitch()); // 2: tync
                pose_data.add(0.0); // 3: ta
                pose_data.add(0.0); // 4: distToCamera
                pose_data.add(0.0); // 5: distToRobot
                pose_data.add(0.5); // 6: ambiguity
            }

            table.getEntry("botpose_wpiblue")
                    .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
            table.getEntry("botpose_orb_wpiblue")
                    .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        }

        table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
        table.getEntry("cl").setDouble(result.getLatencyMillis());
    }
}
