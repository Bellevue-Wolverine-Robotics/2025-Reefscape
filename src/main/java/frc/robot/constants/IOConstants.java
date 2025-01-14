package frc.robot.constants;

public final class IOConstants {
	public static class JoystickPortConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	public static class DriverButtonConstants {
		public static final int kDriveSpeedPreset1Button = 8;
		public static final int kDriveSpeedPreset2Button = 9;
	}

	public static class OperatorButtonConstants {
		// Flywheel
		public static final int kShootNoteButton = 1;
		public static final int kAimAtStageButton = 2;
		public static final int kAimAtTrapButton = 3;

		// Climbing
		public static final int kClimbUpButton = 4;
		public static final int kClimbDownButton = 5;

		// Intake
		// public static final int kStartIntakeButton = 6;
		// public static final int kStopIntakeButton = 7;
		public static final int kFullIntakeCycle = 2;
		public static final int kFullShootCycle = 3;
	}

}