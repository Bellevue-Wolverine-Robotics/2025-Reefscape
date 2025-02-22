package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class VisionConstants {

  // Camera Names
  public static final String kFrontCamera = "front";

  // PID Values
  public static final double kVisionTurnkP = 0.025;
  public static final double kVisionForwardkP = 0.05;

  // FrontCamera
  public static final double kFrontCamHeightMeters = 0.5;
  public static final double kFrontCamRotRadians = Units.degreesToRadians(0);

  // Tag values
  public static final double kTargetHeightMeters = 1.435;
}
