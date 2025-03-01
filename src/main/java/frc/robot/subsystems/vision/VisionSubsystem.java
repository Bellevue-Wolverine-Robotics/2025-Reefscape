package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.MathUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private PhotonCamera frontCamera;
  private PhotonCamera backCamera;
  private ArrayList<PhotonCamera> allCameras;

  private AprilTagFieldLayout fieldLayout;
  PhotonPoseEstimator photonPoseEstimator;

  public VisionSubsystem() {
    frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA);
    backCamera = new PhotonCamera(VisionConstants.BACK_CAMERA);

    allCameras = new ArrayList<>();
    allCameras.add(frontCamera);
    allCameras.add(backCamera);

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    photonPoseEstimator = new PhotonPoseEstimator(
      fieldLayout,
      PoseStrategy.LOWEST_AMBIGUITY,
      VisionConstants.FRONT_CAMERA_TO_ROBOT.inverse()
    );

    if (Robot.isSimulation()) {
      setupSimulation();
    }
  }

  /**
   * Retrieves the AprilTag detection information for the specified fiducial ID.
   *
   * @param fiducialId The AprilTag ID to look for.
   * @return An {@link AprilTagStruct} containing detection data. If the tag is not visible,
   *         targetVisible will remain false.
   */
  public AprilTagStruct getTargetID(int fiducialId) {
    AprilTagStruct aprilTagData = new AprilTagStruct();

    var results = frontCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      return aprilTagData;
    }

    var latestResult = results.get(results.size() - 1);
    if (!latestResult.hasTargets()) {
      return aprilTagData;
    }

    var targetOptional = latestResult
      .getTargets()
      .stream()
      .filter(target -> target.getFiducialId() == fiducialId)
      .filter(
        target ->
          target.getPoseAmbiguity() <= 1 && target.getPoseAmbiguity() != -1
      )
      .findFirst();

    if (!targetOptional.isPresent()) {
      return aprilTagData;
    }

    var target = targetOptional.get();
    // Directly use the raw yaw and camera-to-target transform
    aprilTagData.yaw = target.getYaw();
    aprilTagData.camToTarget = target.getBestCameraToTarget();
    aprilTagData.targetVisible = true;

    return aprilTagData;
  }

  /**
   * Estimates the robot pose from a single camera's vision data.
   *
   * @param camera The PhotonCamera to use for pose estimation
   * @param cameraToRobot The transform from camera to robot center
   * @return Optional containing EstimatedPoseStruct if successful, empty otherwise
   */
  public Optional<EstimatedPoseStruct> estimateRobotPoseVision(
    PhotonCamera camera,
    Transform3d cameraToRobot
  ) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    if (!latestResult.getMultiTagResult().isPresent()) {
      return Optional.empty();
    }

    MultiTargetPNPResult multiTargets = latestResult.getMultiTagResult().get();
    Transform3d fieldToCamera = multiTargets.estimatedPose.best;
    Transform3d fieldToRobot = MathUtils.addTransforms(
      fieldToCamera,
      cameraToRobot
    );
    Pose2d robotPose = new Pose3d(
      fieldToRobot.getTranslation(),
      fieldToRobot.getRotation()
    ).toPose2d();

    List<PhotonTrackedTarget> targets = latestResult.getTargets();
    if (targets.isEmpty()) {
      return Optional.empty();
    }

    double avgDistance = calculateAverageDistance(targets);

    EstimatedPoseStruct estimatedPose = new EstimatedPoseStruct();
    estimatedPose.robotPose = robotPose;
    estimatedPose.estimationTimestamp = latestResult.getTimestampSeconds();
    estimatedPose.distance = avgDistance;

    return Optional.of(estimatedPose);
  }

  /**
   * Calculates the average Euclidean distance to all tracked targets
   *
   * @param targets List of tracked targets
   * @return Average distance to targets
   */
  private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
    double totalDistance = 0.0;
    for (PhotonTrackedTarget target : targets) {
      // Get 3D translation to target in camera space
      Transform3d camToTarget = target.getBestCameraToTarget();
      // Calculate Euclidean distance
      double distance = Math.sqrt(
        Math.pow(camToTarget.getX(), 2) +
        Math.pow(camToTarget.getY(), 2) +
        Math.pow(camToTarget.getZ(), 2)
      );
      totalDistance += distance;
    }
    return totalDistance / targets.size();
  }

  /**
   * Convenience method to estimate pose from the front camera
   *
   * @return Optional containing EstimatedPoseStruct if successful, empty otherwise
   */
  public Optional<EstimatedPoseStruct> estimateRobotPoseVision() {
    return estimateRobotPoseVision(
      frontCamera,
      VisionConstants.FRONT_CAMERA_TO_ROBOT
    );
  }

  /**
   * Estimates robot pose using all available cameras
   *
   * @return ArrayList of all successful pose estimations
   */
  public ArrayList<EstimatedPoseStruct> estimateRobotPoseFromAllCameras() {
    ArrayList<EstimatedPoseStruct> allPoses = new ArrayList<>();

    // Process front camera
    Optional<EstimatedPoseStruct> frontPose = estimateRobotPoseVision(
      frontCamera,
      VisionConstants.FRONT_CAMERA_TO_ROBOT
    );
    frontPose.ifPresent(pose -> allPoses.add(pose));

    // Process back camera
    Optional<EstimatedPoseStruct> backPose = estimateRobotPoseVision(
      backCamera,
      VisionConstants.BACK_CAMERA_TO_ROBOT
    );
    backPose.ifPresent(pose -> allPoses.add(pose));

    return allPoses;
  }

  private VisionSystemSim visionSystemSim;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private SimCameraProperties cameraProperties;
  private PhotonCameraSim frontCameraSim;
  private PhotonCameraSim backCameraSim;

  /**
   * Constructs the VisionSim subsystem.
   *
   * @param robotPoseSupplier A supplier that provides the current robot pose.
   */
  public void setupSimulation() {
    // Initialize the vision system simulation
    visionSystemSim = new VisionSystemSim("main");

    // Load the field layout for AprilTags
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025Reefscape
    );
    visionSystemSim.addAprilTags(aprilTagFieldLayout);

    // Set up simulated camera properties.
    cameraProperties = new SimCameraProperties();
    // Configure a 640x480 camera with a 100 degree diagonal FOV (61.53 deg approx. vertical FOV)
    cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(61.53));
    // Set detection noise parameters (average and standard deviation in pixels)
    cameraProperties.setCalibError(2.3, 0.08);
    // Set camera image capture framerate.
    cameraProperties.setFPS(30);
    // Set average latency and standard deviation in milliseconds.
    cameraProperties.setAvgLatencyMs(50);
    cameraProperties.setLatencyStdDevMs(5);

    frontCameraSim = new PhotonCameraSim(frontCamera, cameraProperties);
    frontCameraSim.enableDrawWireframe(true);

    backCameraSim = new PhotonCameraSim(backCamera, cameraProperties);
    backCameraSim.enableDrawWireframe(true);

    visionSystemSim.addCamera(
      frontCameraSim,
      VisionConstants.FRONT_CAMERA_TO_ROBOT.inverse()
    );

    visionSystemSim.addCamera(
      backCameraSim,
      VisionConstants.BACK_CAMERA_TO_ROBOT.inverse()
    );
  }

  public void updateSim(Pose2d pose) {
    visionSystemSim.update(pose);
  }
}
