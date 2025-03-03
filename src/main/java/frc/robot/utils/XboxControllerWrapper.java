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

  /**
   * Helper method to handle axis readings with proper inversion
   */
  private double getAxisWithInversion(
    int simulationAxis,
    double superValue,
    boolean shouldInvert
  ) {
    double value = Robot.isSimulation()
      ? getRawAxis(simulationAxis)
      : superValue;
    return shouldInvert ? -value : value;
  }

  @Override
  public double getLeftX() {
    return getAxisWithInversion(
      OperatorConstants.kControllerLeftX,
      super.getLeftX(),
      inverted
    );
  }

  @Override
  public double getLeftY() {
    return getAxisWithInversion(
      OperatorConstants.kControllerLeftY,
      super.getLeftY(),
      inverted
    );
  }

  @Override
  public double getRightX() {
    return getAxisWithInversion(
      OperatorConstants.kControllerRightX,
      super.getRightX(),
      rightInverted
    );
  }

  @Override
  public double getRightY() {
    return getAxisWithInversion(
      OperatorConstants.kControllerRightY,
      super.getRightY(),
      inverted
    );
  }

  @Override
  public Trigger rightTrigger() {
    return Robot.isSimulation()
      ? this.button(OperatorConstants.kControllerRightTrigger)
      : super.rightTrigger();
  }

  @Override
  public Trigger y() {
    return Robot.isSimulation() ? this.button(5) : super.y();
  }

  @Override
  public Trigger b() {
    return Robot.isSimulation() ? this.button(2) : super.b();
  }

  @Override
  public Trigger a() {
    return Robot.isSimulation() ? this.button(1) : super.a();
  }

  @Override
  public Trigger x() {
    return Robot.isSimulation() ? this.button(4) : super.x();
  }

  @Override
  public Trigger leftBumper() {
    return Robot.isSimulation() ? this.button(7) : super.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return Robot.isSimulation() ? this.button(8) : super.rightBumper();
  }
}
