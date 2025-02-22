package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera frontCamera;
  private AprilTagFieldLayout fieldLayout;

  public VisionSubsystem() {
    frontCamera = new PhotonCamera(VisionConstants.kFrontCamera);
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
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
            aprilTagResult.distance =
              PhotonUtils.calculateDistanceToTargetMeters(
                // VisionConstants.kFrontCamHeightMeters,
                0.2032,
                // fieldLayout.getTagPose(id).get().getZ(),
                0.762,
                VisionConstants.kFrontCamRotRadians,
                Units.degreesToRadians(target.getPitch())
              );
            // System.out.println("distance from cam: " + aprilTagResult.distance);

            aprilTagResult.targetVisible = true;
          }
        }
      }
    }

    return aprilTagResult;
  }
}
