package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class RobotConstants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    // Running on a real robot
    REAL,
    // Running a physics simulator
    SIM,
    // Replaying from a log file
    REPLAY
  }
}
