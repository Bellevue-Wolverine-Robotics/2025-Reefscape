package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class DriveConstants {
	// CAN IDs
	public static final int kFrontLeftMotorId = 4;
	public static final int kFrontRightMotorId = 2;
	public static final int kBackLeftMotorId = 3;
	public static final int kBackRightMotorId = 1;

	// Motor Configuration
	public static final double kDriveGearRatio = 8.45;
	public static final double kWheelDiameterInches = 6;
	public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
	public static final double kWheelCircumferenceMeters = Units.inchesToMeters(kWheelCircumferenceInches);

	public static final double kTrackWidthInches = 19.75;
	public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

	public static final boolean kLeftInverted = false;
	public static final boolean kRightInverted = true;

	public static final int kCurrentLimit = 30;
	public static final int kVoltageCompensation = 12;

	// Velocity PID configuration
  public static final double realKp = 0.0;
  public static final double realKd = 0.0;
  public static final double realKs = 0.0;
  public static final double realKv = 0.1;

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;
}
