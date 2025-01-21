package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.OperatorConstants;

public class XboxControllerWrapper extends CommandXboxController {
  public XboxControllerWrapper(int port) {
    super(port);
  }

  @Override
  public double getLeftX() {
    if (Robot.isSimulation()) {
      return this.getRawAxis(OperatorConstants.kControllerLeftX);
    }
    return super.getLeftX();
  }

  @Override
  public double getLeftY() {
    if (Robot.isSimulation()) {
      return this.getRawAxis(OperatorConstants.kControllerLeftY);
    }
    return super.getLeftY();
  }

  @Override
  public double getRightX() {
    if (Robot.isSimulation()) {
      return this.getRawAxis(OperatorConstants.kControllerRightX);
    }
    return super.getRightX();
  }

  @Override
  public double getRightY() {
    if (Robot.isSimulation()) {
      return this.getRawAxis(OperatorConstants.kControllerRightY);
    }
    return super.getRightY();
  }

  @Override
  public Trigger rightTrigger() {
    if (Robot.isSimulation()) {
      return this.button(OperatorConstants.kControllerRightTrigger);
    }
    return super.rightTrigger();
  }
}
