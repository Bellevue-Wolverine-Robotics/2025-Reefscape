package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera frontCamera;

  public VisionSubsystem() {
    frontCamera = new PhotonCamera(VisionConstants.kFrontCamera);
  }

  /**
   * Method that provides information of a specific April Tag ID
   *
   * @param id id of the april tag
   * @return the april tag struct
   */
  public AprilTagStruct getTargetID(int id) {
    AprilTagStruct aprilTagResult = new AprilTagStruct();

    var results = frontCamera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 1) {
            aprilTagResult.yaw = target.getYaw();
            aprilTagResult.targetVisible = true;
          }
        }
      }
    }

    return aprilTagResult;
  }

  public AprilTagStruct getTargetID2(int id) {
    AprilTagStruct aprilTagResult = new AprilTagStruct();
            double targetRange = 0.0;

    var results = frontCamera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 1) {
            aprilTagResult.yaw = target.getYaw();

            targetRange =
            PhotonUtils.calculateDistanceToTargetMeters(
            0.5, // Check CAD
            1.435, //Made constants for camera and target height, idk how to put it in this line...
            Units.degreesToRadians(VisionConstants.kDegreesToRadians), // Check CAD
            Units.degreesToRadians(target.getPitch()));

            aprilTagResult.targetVisible = true;
          }
        }
      }
    }
    return aprilTagResult;
  }
}
