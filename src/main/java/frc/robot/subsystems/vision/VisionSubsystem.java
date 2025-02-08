package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

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
}
