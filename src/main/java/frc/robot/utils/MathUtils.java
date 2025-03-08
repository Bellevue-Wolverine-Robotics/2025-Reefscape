package frc.robot.utils;

import edu.wpi.first.math.geometry.*;

public class MathUtils {

  /**
   * Combines two Transform3d objects by adding their translations and rotations.
   * @param first The first Transform3d.
   * @param second The second Transform3d.
   * @return A new Transform3d representing the combined transformation.
   */
  public static Transform3d addTransforms(
    Transform3d first,
    Transform3d second
  ) {
    // Combine translations
    Translation3d combinedTranslation = first
      .getTranslation()
      .plus(second.getTranslation());

    // Combine rotations
    Rotation3d combinedRotation = first
      .getRotation()
      .plus(second.getRotation());

    return new Transform3d(combinedTranslation, combinedRotation);
  }

  /**
   * Subtracts the second Transform3d from the first by subtracting their translations and rotations.
   * @param first The first Transform3d.
   * @param second The second Transform3d.
   * @return A new Transform3d representing the difference between the transformations.
   */
  public static Transform3d subtractTransforms(
    Transform3d first,
    Transform3d second
  ) {
    // Subtract translations
    Translation3d subtractedTranslation = first
      .getTranslation()
      .minus(second.getTranslation());

    // Subtract rotations
    Rotation3d subtractedRotation = first
      .getRotation()
      .minus(second.getRotation());

    return new Transform3d(subtractedTranslation, subtractedRotation);
  }
}
