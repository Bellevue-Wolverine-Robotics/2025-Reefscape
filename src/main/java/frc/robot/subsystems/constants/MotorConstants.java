package frc.robot.subsystems.constants;

/*
 * This class is used to define general global constants for motor controllers. This is useful
 * for empirical values, etc, magic numbers that are used by other constants and subsystems.

 */
public final class MotorConstants {
	public static final double kTalonClosedLoopFullOutput = 1023; // i.e. use kP of 0.25 to get full output if err is
																	// 4096
	public static final double kNEOStallTorque = 2.6;
	public static final double kNEOFreeSpeed = 5676;

}
