package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * April tag struct
 */
public class AprilTagStruct {

  public Boolean targetVisible = false;

  public Double yaw = 0.0;
  public Double distance = 0.0;

  public Transform3d camToTarget = new Transform3d();
}
