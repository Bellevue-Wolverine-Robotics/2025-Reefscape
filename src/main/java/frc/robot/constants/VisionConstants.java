package frc.robot.constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  private VisionConstants() {
    // Prevent instantiation
  }

  // Camera Names
  public static final String FRONT_CAMERA = "front";

  public static final double FRONT_CAMERA_HEIGHT_INCHES = 38;
  public static final double FRONT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(
    FRONT_CAMERA_HEIGHT_INCHES
  );
  public static final double FRONT_CAMERA_FORWARD_INCHES = 1;
  public static final double FRONT_CAMERA_FORWARD_METERS = Units.inchesToMeters(
    FRONT_CAMERA_FORWARD_INCHES
  );
  public static final double FRONT_CAMERA_PITCH_DEGREES = -20;
  public static final double FRONT_CAMERA_PITCH_RADIANS =
    Units.degreesToRadians(FRONT_CAMERA_PITCH_DEGREES);
  public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(
    new Translation3d(
      -FRONT_CAMERA_FORWARD_METERS,
      0,
      -FRONT_CAMERA_HEIGHT_METERS
    ),
    // new Translation3d(),
    new Rotation3d(0, FRONT_CAMERA_PITCH_RADIANS, 0)
  );

  public static final String BACK_CAMERA = "back";

  public static final double BACK_CAMERA_HEIGHT_INCHES = 38;
  public static final double BACK_CAMERA_HEIGHT_METERS = Units.inchesToMeters(
    BACK_CAMERA_HEIGHT_INCHES
  );
  public static final double BACK_CAMERA_FORWARD_INCHES = -4;
  public static final double BACK_CAMERA_FORWARD_METERS = Units.inchesToMeters(
    BACK_CAMERA_FORWARD_INCHES
  );
  public static final double BACK_CAMERA_YAW_DEGREES = 0;
  public static final double BACK_CAMERA_YAW_RADIANS = Units.degreesToRadians(
    BACK_CAMERA_YAW_DEGREES
  );
  public static final Transform3d BACK_CAMERA_TO_ROBOT = new Transform3d(
    new Translation3d(
      -FRONT_CAMERA_FORWARD_METERS,
      0,
      -FRONT_CAMERA_HEIGHT_METERS
    ),
    // new Translation3d(),
    new Rotation3d(0, 0, BACK_CAMERA_YAW_RADIANS)
  );

  // PID controller constants for X-axis
  public static final double X_PID_KP = 6.0;
  public static final double X_PID_KI = 0.0;
  public static final double X_PID_KD = 3.0;
  public static final TrapezoidProfile.Constraints X_CONSTRAINTS =
    new TrapezoidProfile.Constraints(0.5, 0.5);
  public static final double X_TOLERANCE = 0.15;

  // PID controller constants for Y-axis
  public static final double Y_PID_KP = 6.0;
  public static final double Y_PID_KI = 0.0;
  public static final double Y_PID_KD = 3.0;
  public static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
    new TrapezoidProfile.Constraints(0.5, 0.5);
  public static final double Y_TOLERANCE = 0.15;

  // PID controller constants for rotational (theta) axis
  public static final double THETA_PID_KP = 1.0;
  public static final double THETA_PID_KI = 0.0;
  public static final double THETA_PID_KD = 0.0;
  public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
    new TrapezoidProfile.Constraints(5, 4);
  public static final double THETA_TOLERANCE_DEGREES = 1.0;

  // Transformation from tag to goal pose
  public static final Transform3d TAG_TO_GOAL = new Transform3d(
    new Translation3d(1.0, 0.0, 0.0),
    new Rotation3d(0.0, 0.0, Math.PI)
  );

  // Moving average window size for smoothing pose data
  public static final int MOVING_AVERAGE_WINDOW = 60;

  // Base error values (tuned empirically)
  public static final double BASE_X_STD_DEV = 0.2; // meters
  public static final double BASE_Y_STD_DEV = 0.2; // meters
  public static final double BASE_THETA_STD_DEV = Math.toRadians(10); // radians

  // Scale coefficients that increase uncertainty with distance.
  public static final double X_SCALE = 0.2;
  public static final double Y_SCALE = 0.2;
  public static final double THETA_SCALE = Math.toRadians(30);

  /**
   * Calculates the vision measurement standard deviations as a 3x1 matrix based on the
   * estimated robot pose.
   * <p>
   * The standard deviations for x, y, and theta increase linearly with the distance of the
   * robot from the origin.
   *
   * @param estimatedPose the estimated pose of the robot from vision measurements.
   * @return a {@code Matrix<N3, N1>} containing the standard deviations for [x, y, theta].
   */
  public static Matrix<N3, N1> getEstimationStdDevs(Double distance) {
    double xStdDev = BASE_X_STD_DEV + X_SCALE * distance;
    double yStdDev = BASE_Y_STD_DEV + Y_SCALE * distance;
    double thetaStdDev = BASE_THETA_STD_DEV + THETA_SCALE * distance;

    // Use the static fill method from MatBuilder to create a 3x1 matrix.
    return MatBuilder.fill(Nat.N3(), Nat.N1(), xStdDev, yStdDev, thetaStdDev);
  }
}
