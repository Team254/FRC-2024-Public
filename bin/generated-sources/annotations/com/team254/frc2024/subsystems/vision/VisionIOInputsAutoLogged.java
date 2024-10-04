package com.team254.frc2024.subsystems.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("TurretCameraSeesTarget", turretCameraSeesTarget);
    table.put("ElevatorCameraSeesTarget", elevatorCameraSeesTarget);
    table.put("TurretCameraFiducialObservations", turretCameraFiducialObservations);
    table.put("ElevatorCameraFiducialObservations", elevatorCameraFiducialObservations);
    table.put("TurretCameraMegatagPoseEstimate", turretCameraMegatagPoseEstimate);
    table.put("TurretCameraMegatagCount", turretCameraMegatagCount);
    table.put("ElevatorCameraMegatagPoseEstimate", elevatorCameraMegatagPoseEstimate);
    table.put("ElevatorCameraMegatagCount", elevatorCameraMegatagCount);
    table.put("TurretCameraMegatag2PoseEstimate", turretCameraMegatag2PoseEstimate);
    table.put("ElevatorCameraMegatag2PoseEstimate", elevatorCameraMegatag2PoseEstimate);
  }

  @Override
  public void fromLog(LogTable table) {
    turretCameraSeesTarget = table.get("TurretCameraSeesTarget", turretCameraSeesTarget);
    elevatorCameraSeesTarget = table.get("ElevatorCameraSeesTarget", elevatorCameraSeesTarget);
    turretCameraFiducialObservations = table.get("TurretCameraFiducialObservations", turretCameraFiducialObservations);
    elevatorCameraFiducialObservations = table.get("ElevatorCameraFiducialObservations", elevatorCameraFiducialObservations);
    turretCameraMegatagPoseEstimate = table.get("TurretCameraMegatagPoseEstimate", turretCameraMegatagPoseEstimate);
    turretCameraMegatagCount = table.get("TurretCameraMegatagCount", turretCameraMegatagCount);
    elevatorCameraMegatagPoseEstimate = table.get("ElevatorCameraMegatagPoseEstimate", elevatorCameraMegatagPoseEstimate);
    elevatorCameraMegatagCount = table.get("ElevatorCameraMegatagCount", elevatorCameraMegatagCount);
    turretCameraMegatag2PoseEstimate = table.get("TurretCameraMegatag2PoseEstimate", turretCameraMegatag2PoseEstimate);
    elevatorCameraMegatag2PoseEstimate = table.get("ElevatorCameraMegatag2PoseEstimate", elevatorCameraMegatag2PoseEstimate);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.turretCameraSeesTarget = this.turretCameraSeesTarget;
    copy.elevatorCameraSeesTarget = this.elevatorCameraSeesTarget;
    copy.turretCameraFiducialObservations = this.turretCameraFiducialObservations.clone();
    copy.elevatorCameraFiducialObservations = this.elevatorCameraFiducialObservations.clone();
    copy.turretCameraMegatagPoseEstimate = this.turretCameraMegatagPoseEstimate;
    copy.turretCameraMegatagCount = this.turretCameraMegatagCount;
    copy.elevatorCameraMegatagPoseEstimate = this.elevatorCameraMegatagPoseEstimate;
    copy.elevatorCameraMegatagCount = this.elevatorCameraMegatagCount;
    copy.turretCameraMegatag2PoseEstimate = this.turretCameraMegatag2PoseEstimate;
    copy.elevatorCameraMegatag2PoseEstimate = this.elevatorCameraMegatag2PoseEstimate;
    return copy;
  }
}
