package frc.robot.constants;

import frc.utils.PIDUtils;

public final class FlyWheelConstants {
	public static final double kShooterMaxRPM = 5828; // used for calculating FF

	// CAN IDs
	public static final int kArmShoulderId = 8;
	public static final int kArmElbowId = 10;
	public static final int kShooterLeaderId = 11;
	public static final int kShooterFollowerId = 13;
	public static final int kFeederId = 12;

	// PID

	public static final PIDUtils.SparkPIDParams kArmShoulderPid = new PIDUtils.SparkPIDParams(((double) 1 / 19) * 0.3,
			0.001 * ((double) 1 / 19) * 0.3, 0, 0, 0, -1, 1);
	public static final PIDUtils.SparkPIDParams kArmElbowPid = new PIDUtils.SparkPIDParams(((double) 1 / 16) * 0.3,
			(0.001) * ((double) 1 / 16) * 0.3,
			0, 0, 0, -1, 1);

	// Setpoints

	public static final double kArmShoulderTolerance = 1;
	public static final double kArmElbowTolerance = 1;

	// Negative duty cycles correspond to shooting
	// Positive duty cycles correspond to feeding in
	public static final double kShootSpeakerDutyCycleSetpoint = 1;
	public static final double kFeederDutyCycleSetpoint = 1;
	public static final double kFeederDutyCycleSetpointReverse = -1;


	// For the shoulder and elbow, positive encoder and set() values correspond to
	// leaving the
	// robot

	// Negative elbow corresponds to twisting towards the robot, pointing up

	public static final double kSpeakerShoulderSetpoint = 17;
	public static final double kSpeakerElbowSetpoint = -17;

	public static final double kIntakeMakeSpaceShoulderSetpoint = 8;
	public static final double kIntakeMakeSpaceElbowSetpoint = 0;

	public static final double kIntakeClimbShoulderSetpoint = 27.0;
	public static final double kIntakeClimbElbowSetpoint = -0.8;

	public static final double kAmpShoulderSetpoint = 0;
	public static final double kAmpElbowSetpoint = 0;

	public static final double kIntakeReceiveShoulderSetpoint = 6.0;
	public static final double kIntakeReceiveElbowSetpoint = -0.8;

	public static final double kIntakeShooterShoulderSetpoint = 10.2;
	public static final double kIntakeShooterElbowSetpoint = -10.5;

	public static final int kNoteSwitchDIOPort = 6;

}
