package frc.robot.constants;

import frc.utils.PIDUtils.ArmFFParams;
import frc.utils.PIDUtils.SparkPIDParams;
import frc.utils.PIDUtils.TalonPIDParams;;

public final class IntakeConstants {
	public static final int kFeederMotorId = 7;
	public static final int kArmMotorId = 9;

	public static final int kSmartCurrentLimit = 30;

	public static final double kArmPositionConversionFactor = 1.0; // depends on gear ratio and diameter

	public static final double kFeederDutyCycle = -1;
	public static final double kFeedIntoFlywheelDutyCycle = 1;

	public static final SparkPIDParams kIntakeArmDeployPIDParams = new SparkPIDParams(((double) 1 / 8) * 0.2, 0.0, 0.0,
			0.0, 0.0,
			-1, 1.0);
	public static final SparkPIDParams kIntakeArmStowPIDParams = new SparkPIDParams(((double) 1 / 8) * 0.4, 0.0, 0.0,
			0.0, 0.0,
			-1, 1.0);

	// Positive values correspond to deploying the intake arm
	public static final double kIntakeArmSetpointStow = 0.0;
	public static final double kIntakeArmSetpointDeploy = 7.75;

	public static final ArmFFParams kIntakeArmFFParams = new ArmFFParams(0, 0, 0, 0);

	public static final int kNoteSwitchDIOPort = 7;

	public static final double kIntakeArmStowTolerance = 0.5;

}
