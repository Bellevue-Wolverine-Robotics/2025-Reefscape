package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.OperatorConstants;

public class XboxControllerWrapper extends CommandXboxController {

  private final boolean inverted;
  private final boolean rightInverted;

  public XboxControllerWrapper(
    int port,
    boolean inverted,
    boolean rightInverted
  ) {
    super(port);
    this.inverted = inverted;
    this.rightInverted = rightInverted;
  }

  @Override
  public double getLeftX() {
    double value = Robot.isSimulation()
      ? this.getRawAxis(OperatorConstants.kControllerLeftX)
      : super.getLeftX();
    return inverted ? -value : value;
  }

  @Override
  public double getLeftY() {
    double value = Robot.isSimulation()
      ? this.getRawAxis(OperatorConstants.kControllerLeftY)
      : super.getLeftY();
    return inverted ? -value : value;
  }

  @Override
  public double getRightX() {
    double value = Robot.isSimulation()
      ? this.getRawAxis(OperatorConstants.kControllerRightX)
      : super.getRightX();
    return rightInverted ? -value : value;
  }

  @Override
  public double getRightY() {
    double value = Robot.isSimulation()
      ? this.getRawAxis(OperatorConstants.kControllerRightY)
      : super.getRightY();
    return inverted ? -value : value;
  }

  @Override
  public Trigger rightTrigger() {
    return Robot.isSimulation()
      ? this.button(OperatorConstants.kControllerRightTrigger)
      : super.rightTrigger();
  }
}
