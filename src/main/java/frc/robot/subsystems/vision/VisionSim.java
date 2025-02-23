package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionSim extends VisionSubsystem {

  private VisionSystemSim visionSim = new VisionSystemSim("main");
  private TargetModel targetModel = TargetModel.kAprilTag36h11;

  private final AprilTagFieldLayout aprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private SimCameraProperties cameraProp = new SimCameraProperties();

  // The PhotonCamera used in the real robot code.
  private PhotonCamera frontCamera = new PhotonCamera("front");

  // The simulation of this camera. Its values used in real robot code will be
  // updated.
  private PhotonCameraSim cameraSim = new PhotonCameraSim(
    frontCamera,
    cameraProp
  );

  private Supplier<Pose2d> robotPose;

  // Low-pass filter parameters for yaw (adjust alpha as needed)
  private double filteredYaw = 0;
  private Transform3d filteredCamToTag = new Transform3d();
  private final double alpha = 1; // smaller alpha means heavier filtering

  public VisionSim(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(61.53));
    // Approximate detection noise with average and standard deviation error in
    // pixels.
    cameraProp.setCalibError(2.3, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop
    // rate).
    cameraProp.setFPS(30);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(5);

    // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
    // pose,
    // (Robot pose is considered the center of rotation at the floor level, or Z =
    // 0)
    Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
    // and pitched 15 degrees up.
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(0), 0);
    Transform3d robotToCamera = new Transform3d(
      robotToCameraTrl,
      robotToCameraRot
    );

    // Add this camera to the vision system simulation with the given
    // robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(true);

    visionSim.addAprilTags(aprilTagFieldLayout);
  }

  @Override
  public void periodic() {
    visionSim.update(robotPose.get());
  }

  @Override
  public AprilTagStruct getTargetID(int id) {
    AprilTagStruct aprilTagResult = new AprilTagStruct();

    var results = frontCamera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        var targetOpt = result
          .getTargets()
          .stream()
          .filter(t -> t.getFiducialId() == id)
          .filter(t -> t.getPoseAmbiguity() <= 1 && t.getPoseAmbiguity() != -1)
          .findFirst();

        if (targetOpt.isPresent()) {
          var target = targetOpt.get();

          // Apply low-pass filter on yaw:
          double newYaw = target.getYaw();
          filteredYaw = alpha * newYaw + (1 - alpha) * filteredYaw;
          aprilTagResult.yaw = filteredYaw;

          Transform3d newCamToTag = target.getBestCameraToTarget();
          filteredCamToTag = new Transform3d(
            new Translation3d(
              alpha * newCamToTag.getX() +
              (1 - alpha) * filteredCamToTag.getTranslation().getX(),
              alpha * newCamToTag.getY() +
              (1 - alpha) * filteredCamToTag.getTranslation().getY(),
              alpha * newCamToTag.getZ() +
              (1 - alpha) * filteredCamToTag.getTranslation().getZ()
            ),
            new Rotation3d(
              alpha * newCamToTag.getRotation().getX() +
              (1 - alpha) * filteredCamToTag.getRotation().getX(),
              alpha * newCamToTag.getRotation().getY() +
              (1 - alpha) * filteredCamToTag.getRotation().getY(),
              alpha * newCamToTag.getRotation().getZ() +
              (1 - alpha) * filteredCamToTag.getRotation().getZ()
            )
          );
          // You can similarly filter other values if needed, e.g., the camera-to-target
          // transform.
          aprilTagResult.camToTarget = filteredCamToTag;
          aprilTagResult.targetVisible = true;
        }
      }
    }

    return aprilTagResult;
  }
}
