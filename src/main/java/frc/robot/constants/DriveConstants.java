package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {

  // CAN IDs
  public static final int kFrontLeftMotorId = 4;
  public static final int kFrontRightMotorId = 2;
  public static final int kBackLeftMotorId = 3;
  public static final int kBackRightMotorId = 1;

  // Wheel Configuration
  public static final double kDriveGearRatio = 8.45;
  public static final double kWheelDiameterInches = 6;
  public static final double kWheelRadiusMeters = Units.inchesToMeters(
    kWheelDiameterInches / 2
  );
  public static final double kWheelCircumferenceInches =
    kWheelDiameterInches * Math.PI;
  public static final double kWheelCircumferenceMeters = Units.inchesToMeters(
    kWheelCircumferenceInches
  );

  public static final double kTrackWidthInches = 19.75;
  public static final double kTrackWidthMeters = Units.inchesToMeters(
    kTrackWidthInches
  );

  // Motor Configuration
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = true;

  public static final int kCurrentLimit = 20;
  public static final int kVoltageCompensation = 12;

  public static final double kMaxSpeedMetersPerSec = 5;

  public static final DCMotor gearbox = DCMotor.getCIM(2);

  /**
   * Velocity PID configuration, common variables:
   * P - proportional
   * I - integral
   * D - derivative
   *
   * S - static friction
   * V - velocity
   * A - acceleration
   * G - gravity
   */
  public static final double realKp = 0.0;
  public static final double realKd = 0.0;
  public static final double realKs = 0.0;
  public static final double realKv = 0.1;

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig = new RobotConfig(
    robotMassKg,
    robotMOI,
    new ModuleConfig(
      kWheelRadiusMeters,
      kMaxSpeedMetersPerSec,
      wheelCOF,
      gearbox.withReduction(kDriveGearRatio),
      kCurrentLimit,
      2
    ),
    kTrackWidthMeters
  );

  // Other

  // should be inbetween -0.15 and 0.15
  public static final double kAngularVelocityCompensation = -0.1;

  public static final double PATH_PLANNER_LINEAR_KP = 6.0;
  public static final double PATH_PLANNER_ANGULAR_KP = 8.0;
}
