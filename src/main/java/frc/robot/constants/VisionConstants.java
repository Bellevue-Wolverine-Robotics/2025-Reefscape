package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  // Camera Names
  public static final String kFrontCamera = "front";

  // PID Values
  public static final double kVisionTurnkP = 0.25;
  public static final double kVisionForwardkP = 0.05;

  // FrontCamera
  public static final double kFrontCamHeightMeters = 0.5;
  public static final double kFrontCamRotRadians = Units.degreesToRadians(0);

  // Tag values
  public static final double kTargetHeightMeters = 1.435;

  // PID controller constants for X-axis
  public static final double X_PID_KP = 8.0;
  public static final double X_PID_KI = 0.0;
  public static final double X_PID_KD = 3.0;
  public static final TrapezoidProfile.Constraints X_CONSTRAINTS =
    new TrapezoidProfile.Constraints(4, 3);
  public static final double X_TOLERANCE = 0.15;

  // PID controller constants for Y-axis
  public static final double Y_PID_KP = 8.0;
  public static final double Y_PID_KI = 0.0;
  public static final double Y_PID_KD = 3.0;
  public static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
    new TrapezoidProfile.Constraints(4, 3);
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
  public static final int MOVING_AVERAGE_WINDOW = 5;
}
