package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private static class Camera {
        private final PhotonCamera photonCamera;
        private final PhotonPoseEstimator photonPoseEstimator;
        private final VisionConstants.CameraProperties properties;
        private PhotonCameraSim cameraSim;

        public Camera(VisionConstants.CameraProperties properties) {
            this.properties = properties;
            photonCamera = new PhotonCamera(properties.name);
            photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT, properties.robotToCamera);
        }

        public void setupSimulation(VisionSystemSim visionSim) {
            SimCameraProperties cameraProp = new SimCameraProperties();

            cameraProp.setCalibration(
                VisionConstants.CAMERA_RESOLUTION_WIDTH,
                VisionConstants.CAMERA_RESOLUTION_HEIGHT,
                Rotation2d.fromDegrees(VisionConstants.CAMERA_DIAGONAL_FOV)
            );

            cameraProp.setCalibError(VisionConstants.CAMERA_AVERAGE_ERROR_PIXEL, VisionConstants.CAMERA_ERROR_STD_DEV_PIXEL);
            cameraProp.setFPS(VisionConstants.CAMERA_FPS);
            cameraProp.setAvgLatencyMs(VisionConstants.CAMERA_LATENCY_MS);
            cameraProp.setLatencyStdDevMs(VisionConstants.CAMERA_LATENCY_STD_DEV_MS);

            cameraSim = new PhotonCameraSim(photonCamera, cameraProp);
            visionSim.addCamera(cameraSim, properties.robotToCamera);
            cameraSim.enableDrawWireframe(true);
        }

        public Optional<PoseEstimate> estimatePose() {
            List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

            if (results.isEmpty()) {
                return Optional.empty();
            }

            PhotonPipelineResult result = results.get(results.size() - 1);
            Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);

            if (estimatedPose.isEmpty()) {
                estimatedPose = photonPoseEstimator.estimateLowestAmbiguityPose(result);

                if (estimatedPose.isEmpty()) {
                    return Optional.empty();
                }
            }


            Matrix<N3, N1> stdDevs = calculateEstimationStdDevs(estimatedPose.get(), result.getTargets());
            return Optional.of(new PoseEstimate(estimatedPose.get(), stdDevs));
        }

        /**
         * Calculates standard deviations. This algorithm is a heuristic that creates dynamic standard
         * deviations based on number of tags, estimation strategy, and distance from the tags.
         *
         * @param estimatedPose The estimated pose to guess standard deviations for.
         * @param targets All targets in this camera frame
         * @return Estimated standard deviations.
         */
        private Matrix<N3, N1> calculateEstimationStdDevs(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
            var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                Optional<Pose3d> tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
                    estimatedPose.estimatedPose.toPose2d().getTranslation()
                );
            }

            if (numTags == 0) {
                return estStdDevs;
            }

            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
            }
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > VisionConstants.SINGLE_TAG_DISTANCE_THRESHOLD) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
            else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / VisionConstants.STD_DEVS_SCALING_FACTOR));
            };

            return estStdDevs;
        }
    }

    private static class PoseEstimate {
        public final EstimatedRobotPose pose;
        public final Matrix<N3, N1> stdDevs;

        public PoseEstimate(EstimatedRobotPose pose, Matrix<N3,N1> stdDevs) {
            this.pose = pose;
            this.stdDevs = stdDevs;
        }
    }

    private final List<Camera> cameras = new ArrayList<Camera>();
    private final SwerveSubsystem swerveSubsystem;
    private VisionSystemSim visionSim;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        for (VisionConstants.CameraProperties cameraProperties : VisionConstants.CAMERAS) {
            cameras.add(new Camera(cameraProperties));
        }

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);

            for (Camera camera: cameras) {
                camera.setupSimulation(visionSim);
            }
        }
    }

    @Override
    public void periodic() {
        for (Camera camera: cameras) {
            Optional<PoseEstimate> cameraEstimate = camera.estimatePose();

            cameraEstimate.ifPresent(estimate -> swerveSubsystem.addVisionMeasurement(
                estimate.pose.estimatedPose.toPose2d(),
                estimate.pose.timestampSeconds,
                estimate.stdDevs
            ));
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(swerveSubsystem.getSimulationPose());
    }
}
