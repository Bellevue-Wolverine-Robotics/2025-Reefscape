// Copyright 2021-2025 FRC 6328
// from http://github.com/Mechanical-Advantage

package frc.robot.interfaces;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for running tank drivetrain motors and logging motor values
 */
public interface DriveIO {
  /** Class storing the left and right motor states */
  @AutoLog
  public static class DriveIOInputs {
    /** position of left motor */
    public double leftPositionRad = 0.0;
    /** speed of left motor */
    public double leftVelocityRadPerSec = 0.0;
    /** voltage of left motor */
    public double leftAppliedVolts = 0.0;
    /** current of left motor */
    public double[] leftCurrentAmps = new double[] {};

    /** position of right motor */
    public double rightPositionRad = 0.0;
    /** speed of right motor */
    public double rightVelocityRadPerSec = 0.0;
    /** voltage of right motor */
    public double rightAppliedVolts = 0.0;
    /** current of right motor */
    public double[] rightCurrentAmps = new double[] {};
  }

  /**
   * Updates the set of loggable inputs
   *
   * @param inputs The DriveIOInputs to update
   */
  public default void updateInputs(DriveIOInputs inputs) {
  }

  /**
   * Run open loop at the specified voltage.
   *
   * @param leftVolts  voltage for the left motor
   * @param rightVolts voltage for the right motor
   */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  /**
   * Run closed loop at the specified velocity.
   *
   * @param leftRadPerSec  speed for left motor, in rad
   * @param rightRadPerSec speed for right motor, in rad
   * @param leftFFVolts    Feedfoward voltage for left motor
   * @param rightFFVolts   Feedforard voltage for right motor
   */
  public default void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
  }
}
