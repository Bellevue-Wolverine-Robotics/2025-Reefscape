package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

/**
 * Utility class for retrieving field-relative poses of April Tags
 * for the 2025 Reefscape field.
 */
public class AprilTagUtils {

  // Get the official 2025 Reefscape field layout
  private static final AprilTagFieldLayout fieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  /**
   * Gets the field-relative pose of an April Tag with the specified ID.
   *
   * @param id The ID of the April Tag to retrieve
   * @return The Pose2d of the April Tag, or null if the ID is not recognized
   */
  public static Pose2d getAprilTagPose(int id) {
    Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(id);
    return tagPose3d.map(pose3d -> pose3d.toPose2d()).orElse(null);
  }

  /**
   * Checks if the specified April Tag ID exists in the field layout.
   *
   * @param id The ID to check
   * @return True if the April Tag ID exists, false otherwise
   */
  public static boolean isValidAprilTagId(int id) {
    return fieldLayout.getTagPose(id).isPresent();
  }

  /**
   * Gets all April Tag IDs in the field layout.
   *
   * @return An array of all valid April Tag IDs
   */
  public static Integer[] getAllAprilTagIds() {
    return fieldLayout
      .getTags()
      .stream()
      .map(tagData -> tagData.ID)
      .toArray(Integer[]::new);
  }
}
