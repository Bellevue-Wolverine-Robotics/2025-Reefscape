package frc.robot.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.VisionConstants;

/**
 * Encapsulates the PID controllers used for chasing the target.
 */
public class TagPidControllers {

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  /**
   * Constructs the PID controllers using parameters from VisionConstants.
   */
  public TagPidControllers() {
    xController = new ProfiledPIDController(
      VisionConstants.X_PID_KP,
      VisionConstants.X_PID_KI,
      VisionConstants.X_PID_KD,
      VisionConstants.X_CONSTRAINTS
    );
    yController = new ProfiledPIDController(
      VisionConstants.Y_PID_KP,
      VisionConstants.Y_PID_KI,
      VisionConstants.Y_PID_KD,
      VisionConstants.Y_CONSTRAINTS
    );
    thetaController = new ProfiledPIDController(
      VisionConstants.THETA_PID_KP,
      VisionConstants.THETA_PID_KI,
      VisionConstants.THETA_PID_KD,
      VisionConstants.THETA_CONSTRAINTS
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Resets all PID controllers to the current robot pose.
   *
   * @param robotPose The current pose of the robot.
   */
  public void reset(Pose2d robotPose) {
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    thetaController.reset(robotPose.getRotation().getRadians());
  }

  /**
   * Sets the goal for the X and Y PID controllers based on the desired target pose.
   *
   * @param goalPose The target goal pose.
   */
  public void setGoals(Pose2d goalPose) {
    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
  }

  /**
   * Calculates the control output for the X-axis.
   *
   * @param currentX The current X position of the robot.
   * @param goalX The desired X position.
   * @return The computed control output for the X-axis.
   */
  public double calculateX(double currentX, double goalX) {
    double output = xController.calculate(currentX);
    if (Math.abs(goalX - currentX) < VisionConstants.X_TOLERANCE) {
      output = 0;
    }
    return output;
  }

  /**
   * Calculates the control output for the Y-axis.
   *
   * @param currentY The current Y position of the robot.
   * @param goalY The desired Y position.
   * @return The computed control output for the Y-axis.
   */
  public double calculateY(double currentY, double goalY) {
    double output = yController.calculate(currentY);
    if (Math.abs(goalY - currentY) < VisionConstants.Y_TOLERANCE) {
      output = 0;
    }
    return output;
  }

  /**
   * Calculates the rotational control output based on the tag's yaw error.
   * The controller is set so that the desired yaw error is always 0,
   * which causes the robot to continuously face the tag.
   *
   * @param tagYaw The yaw error (in radians) from the tag (positive means tag is to one side).
   * @return The computed control output for rotation.
   */
  public double calculateRotation(double currentHeading, double tagYaw) {
    // Compute the measurement as (tagYaw - currentHeading)
    double measurement = tagYaw - currentHeading;
    // Set the setpoint to zero so that error = 0 - measurement.
    // WARNING: FOR SOME REASON 0.1 IS THE ONLY VALUE THAT WORKS..?
    thetaController.setGoal(0);
    double output = thetaController.calculate(measurement * 0.1);
    // Optionally zero out output if error is within tolerance.
    if (
      Math.abs(measurement) <
      Units.degreesToRadians(VisionConstants.THETA_TOLERANCE_DEGREES)
    ) {
      output = 0;
    }
    return output;
  }
}
