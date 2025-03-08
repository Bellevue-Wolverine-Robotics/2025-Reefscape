package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.AprilTagUtils;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Constants class containing AprilTag positions for the 2025 Reefscape game.
 * Handles both BLUE and RED alliance sides based on FMS data.
 * Uses suppliers to defer evaluation until runtime.
 */
public final class AprilTagConstants {

  // Standard offset transforms
  private static final Transform2d STANDARD_APPROACH_TRANSFORM =
    new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI));

  private static final Transform2d CORAL_STATION_APPROACH_TRANSFORM =
    new Transform2d(new Translation2d(1, 0), new Rotation2d(0));

  // Tag IDs for BLUE alliance side
  private static final int BLUE_BOTTOM_RIGHT_TAG_ID = 17;
  private static final int BLUE_BOTTOM_TAG_ID = 18;
  private static final int BLUE_BOTTOM_LEFT_TAG_ID = 19;
  private static final int BLUE_TOP_LEFT_TAG_ID = 20;
  private static final int BLUE_TOP_TAG_ID = 21;
  private static final int BLUE_TOP_RIGHT_TAG_ID = 22;
  private static final int BLUE_RIGHT_CORAL_STATION_TAG_ID = 12;
  private static final int BLUE_LEFT_CORAL_STATION_TAG_ID = 13;

  // Tag IDs for RED alliance side
  private static final int RED_BOTTOM_RIGHT_TAG_ID = 8;
  private static final int RED_BOTTOM_TAG_ID = 7;
  private static final int RED_BOTTOM_LEFT_TAG_ID = 6;
  private static final int RED_TOP_LEFT_TAG_ID = 11;
  private static final int RED_TOP_TAG_ID = 10;
  private static final int RED_TOP_RIGHT_TAG_ID = 9;
  private static final int RED_RIGHT_CORAL_STATION_TAG_ID = 2;
  private static final int RED_LEFT_CORAL_STATION_TAG_ID = 1;

  /**
   * Creates a supplier that returns the appropriate tag ID based on the alliance color from FMS
   * @param blueTagId The tag ID to use when on blue alliance
   * @param redTagId The tag ID to use when on red alliance
   * @return A supplier that provides the alliance-specific tag ID when evaluated
   */
  private static Supplier<Integer> getAllianceSpecificTagIdSupplier(
    int blueTagId,
    int redTagId
  ) {
    return () -> {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      return alliance.isPresent() && alliance.get() == Alliance.Red
        ? redTagId
        : blueTagId;
    };
  }

  /**
   * Returns a supplier for the bottom tag approach pose
   */
  public static Supplier<Pose2d> getBottomTagApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(BLUE_BOTTOM_TAG_ID, RED_BOTTOM_TAG_ID),
        STANDARD_APPROACH_TRANSFORM
      );
  }

  /**
   * Returns a supplier for the top left tag approach pose
   */
  public static Supplier<Pose2d> getTopLeftTagApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(
          BLUE_TOP_LEFT_TAG_ID,
          RED_TOP_LEFT_TAG_ID
        ),
        STANDARD_APPROACH_TRANSFORM
      );
  }

  /**
   * Returns a supplier for the top tag approach pose
   */
  public static Supplier<Pose2d> getTopTagApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(BLUE_TOP_TAG_ID, RED_TOP_TAG_ID),
        STANDARD_APPROACH_TRANSFORM
      );
  }

  /**
   * Returns a supplier for the top right tag approach pose
   */
  public static Supplier<Pose2d> getTopRightTagApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(
          BLUE_TOP_RIGHT_TAG_ID,
          RED_TOP_RIGHT_TAG_ID
        ),
        STANDARD_APPROACH_TRANSFORM
      );
  }

  /**
   * Returns a supplier for the bottom right tag approach pose
   */
  public static Supplier<Pose2d> getBottomRightTagApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(
          BLUE_BOTTOM_RIGHT_TAG_ID,
          RED_BOTTOM_RIGHT_TAG_ID
        ),
        STANDARD_APPROACH_TRANSFORM
      );
  }

  /**
   * Returns a supplier for the bottom left tag approach pose
   */
  public static Supplier<Pose2d> getBottomLeftTagApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(
          BLUE_BOTTOM_LEFT_TAG_ID,
          RED_BOTTOM_LEFT_TAG_ID
        ),
        STANDARD_APPROACH_TRANSFORM
      );
  }

  /**
   * Returns a supplier for the right coral station approach pose
   */
  public static Supplier<Pose2d> getRightCoralStationApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(
          BLUE_RIGHT_CORAL_STATION_TAG_ID,
          RED_RIGHT_CORAL_STATION_TAG_ID
        ),
        CORAL_STATION_APPROACH_TRANSFORM
      );
  }

  /**
   * Returns a supplier for the left coral station approach pose
   */
  public static Supplier<Pose2d> getLeftCoralStationApproachPoseSupplier() {
    return () ->
      getPoseFromTagSupplier(
        getAllianceSpecificTagIdSupplier(
          BLUE_LEFT_CORAL_STATION_TAG_ID,
          RED_LEFT_CORAL_STATION_TAG_ID
        ),
        CORAL_STATION_APPROACH_TRANSFORM
      );
  }

  /**
   * Helper method to get a pose from an AprilTag ID supplier with the specified transform applied
   * @param tagIdSupplier The supplier for the AprilTag ID
   * @param transform The transform to apply to the AprilTag pose
   * @return The pose to drive to, with the transform applied
   */
  private static Pose2d getPoseFromTagSupplier(
    Supplier<Integer> tagIdSupplier,
    Transform2d transform
  ) {
    int tagId = tagIdSupplier.get();
    return AprilTagUtils.getAprilTagPose(tagId).transformBy(transform);
  }
}
