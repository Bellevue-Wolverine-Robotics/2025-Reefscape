package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.controllers.TagPidControllers;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.AprilTagStruct;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.MovingAverage;
import java.util.function.Supplier;

/**
 * Command that uses vision processing to chase an AprilTag.
 */
public class ChaseTagCommand extends Command {

  private final Supplier<Pose2d> poseProvider;
  private final TagPidControllers pidControllers;
  private final MovingAverage xMovingAverage;
  private final MovingAverage yMovingAverage;
  private final MovingAverage thetaMovingAverage;

  private AprilTagStruct lastDetectedTag;
  private final VisionSubsystem visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final int targetId;

  /**
   * Constructs a new ChaseTagCommand.
   *
   * @param visionSubsystem the vision subsystem
   * @param swerveSubsystem the swerve drive subsystem
   * @param targetId        the target AprilTag ID to chase
   */
  public ChaseTagCommand(
    VisionSubsystem visionSubsystem,
    SwerveSubsystem swerveSubsystem,
    int targetId
  ) {
    this.visionSubsystem = visionSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.targetId = targetId;

    this.poseProvider = swerveSubsystem::getPose;
    this.pidControllers = new TagPidControllers();
    this.xMovingAverage = new MovingAverage(
      VisionConstants.MOVING_AVERAGE_WINDOW
    );
    this.yMovingAverage = new MovingAverage(
      VisionConstants.MOVING_AVERAGE_WINDOW
    );
    this.thetaMovingAverage = new MovingAverage(
      VisionConstants.MOVING_AVERAGE_WINDOW
    );

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = poseProvider.get();
    pidControllers.reset(currentPose);
    lastDetectedTag = null;
  }

  @Override
  public void execute() {
    Pose2d currentPose2d = poseProvider.get();
    Pose3d currentPose3d = new Pose3d(
      currentPose2d.getX(),
      currentPose2d.getY(),
      0.0,
      new Rotation3d(0.0, 0.0, currentPose2d.getRotation().getRadians())
    );

    AprilTagStruct detectedTag = visionSubsystem.getTargetID(targetId);

    // If no tag has been detected yet and the current reading shows no target,
    // lock the swerve drive.
    if (lastDetectedTag == null && !detectedTag.targetVisible) {
      swerveSubsystem.lock();
    } else if (detectedTag.targetVisible) {
      lastDetectedTag = detectedTag;
    }

    if (lastDetectedTag != null) {
      Transform3d cameraToTarget = lastDetectedTag.camToTarget;
      Pose3d targetPose = currentPose3d.transformBy(cameraToTarget);

      // Compute the raw goal pose by applying the tag-to-goal transformation.
      Pose2d rawGoalPose = targetPose
        .transformBy(VisionConstants.TAG_TO_GOAL)
        .toPose2d();
      Pose2d goalPose = new Pose2d(
        xMovingAverage.next(rawGoalPose.getX()),
        yMovingAverage.next(rawGoalPose.getY()),
        new Rotation2d(0)
      );

      // Update translational PID goals
      pidControllers.setGoals(goalPose);

      double xSpeed = pidControllers.calculateX(
        currentPose3d.getX(),
        goalPose.getX()
      );
      double ySpeed = pidControllers.calculateY(
        currentPose3d.getY(),
        goalPose.getY()
      );

      // NEW: Use the tag's yaw directly to control rotation.
      // It is assumed that lastDetectedTag.yaw is in radians.
      double thetaSpeed = pidControllers.calculateRotation(
        currentPose2d.getRotation().getRadians(),
        lastDetectedTag.yaw // Ensure tag.yaw is in radians.
      );

      swerveSubsystem.drive(
        new Translation2d(xSpeed, ySpeed),
        thetaSpeed,
        false
      );
    }
  }
}
