package com.team254.frc2024.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    class VisionIOInputs {
        public boolean turretCameraSeesTarget;

        public boolean elevatorCameraSeesTarget;

        public FiducialObservation[] turretCameraFiducialObservations;

        public FiducialObservation[] elevatorCameraFiducialObservations;

        public MegatagPoseEstimate turretCameraMegatagPoseEstimate;

        public int turretCameraMegatagCount;

        public MegatagPoseEstimate elevatorCameraMegatagPoseEstimate;

        public int elevatorCameraMegatagCount;

        public MegatagPoseEstimate turretCameraMegatag2PoseEstimate;

        public MegatagPoseEstimate elevatorCameraMegatag2PoseEstimate;
    }

    void readInputs(VisionIOInputs inputs);

    void pollNetworkTables();
}
