package frc.robot.interfaces;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a hardware abstraction layer for a gyroscope that
 * supports logging with AdvantageKit.
 */
public interface GyroIO {
  /** Class for storing gyro states */
  @AutoLog
  public static class GyroIOInputs {
    /** Is true when the gyro is connected to the code (roboRIO) */
    public boolean connected = false;
    /** The heading of the gyro */
    public Rotation2d yawPosition = new Rotation2d();
    /** How fast the gyro turning */
    public double yawVelocityRadPerSec = 0.0;
  }

  /**
   * Updates the set of loggable inputs
   *
   * @param inputs The GyroIOInputs to update
   */
  public default void updateInputs(GyroIOInputs inputs) {
  }
}
