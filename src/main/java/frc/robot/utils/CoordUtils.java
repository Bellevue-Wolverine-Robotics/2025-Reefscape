package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Utility methods for converting between 3D and 2D poses, properly accounting for
 * camera angles and z-axis components.
 */
public class CoordUtils {

  /**
   * Converts a 3D pose to a 2D pose while properly accounting for the z-component
   * and any pitch/roll angles in the rotation.
   *
   * @param pose3d The 3D pose to convert
   * @return A 2D pose with x and y values properly projected onto the ground plane
   */
  public static Pose2d pose3dToPose2d(Pose3d pose3d) {
    // Get the components of the 3D pose
    Translation3d translation = pose3d.getTranslation();
    Rotation3d rotation = pose3d.getRotation();

    // Extract just the yaw component for the 2D rotation
    Rotation2d rotation2d = new Rotation2d(rotation.getZ());

    // Project the translation onto the XY plane
    Translation2d translation2d = new Translation2d(
      translation.getX(),
      translation.getY()
    );

    return new Pose2d(translation2d, rotation2d);
  }

  /**
   * Converts a 3D transform to a 2D pose while accounting for pitch and roll.
   * This is useful when working with camera-to-field transformations.
   *
   * @param transform3d The 3D transform to convert
   * @return A 2D pose with x and y values properly projected onto the ground plane
   */
  public static Pose2d transform3dToPose2d(Transform3d transform3d) {
    // Convert the transform to a pose first
    Pose3d pose3d = new Pose3d().plus(transform3d);
    return pose3dToPose2d(pose3d);
  }

  /**
   * Correctly projects a 3D point from a pitched camera onto the ground plane.
   * Accounts for both the height and pitch of the camera.
   *
   * @param fieldToCamera The transform from field to camera
   * @param cameraToRobot The transform from camera to robot center
   * @return A 2D pose representing the robot's position on the field
   */
  public static Pose2d projectCameraPoseToGround(
    Transform3d fieldToCamera,
    Transform3d cameraToRobot
  ) {
    // Combine the transforms to get field-to-robot transform
    Transform3d fieldToRobot = new Transform3d(
      fieldToCamera
        .getTranslation()
        .plus(
          cameraToRobot.getTranslation().rotateBy(fieldToCamera.getRotation())
        ),
      fieldToCamera.getRotation().plus(cameraToRobot.getRotation())
    );

    // Extract the components
    Translation3d translation = fieldToRobot.getTranslation();
    Rotation3d rotation = fieldToRobot.getRotation();

    // Keep only the yaw component for the 2D rotation
    Rotation2d yaw = new Rotation2d(rotation.getZ());

    // Project onto the XY plane
    Translation2d translation2d = new Translation2d(
      translation.getX(),
      translation.getY()
    );

    return new Pose2d(translation2d, yaw);
  }
}
