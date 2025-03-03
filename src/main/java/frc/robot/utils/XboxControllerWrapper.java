package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.OperatorConstants;

public class XboxControllerWrapper extends CommandGenericHID {

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
    int realAxis,
    boolean shouldInvert
  ) {
    int axisToUse = Robot.isSimulation() ? simulationAxis : realAxis;
    double value = getRawAxis(axisToUse);
    return shouldInvert ? -value : value;
  }

  public double getLeftX() {
    return getAxisWithInversion(
      OperatorConstants.kControllerLeftX,
      0, // Standard mapping for real hardware
      inverted
    );
  }

  public double getLeftY() {
    return getAxisWithInversion(
      OperatorConstants.kControllerLeftY,
      1, // Standard mapping for real hardware
      inverted
    );
  }

  public double getRightX() {
    return getAxisWithInversion(
      OperatorConstants.kControllerRightX,
      4, // Standard mapping for real hardware
      rightInverted
    );
  }

  public double getRightY() {
    return getAxisWithInversion(
      OperatorConstants.kControllerRightY,
      5, // Standard mapping for real hardware
      inverted
    );
  }

  public Trigger rightTrigger() {
    return axisGreaterThan(4, 0.5); // Standard right trigger axis
  }

  public Trigger y() {
    return Robot.isSimulation() ? button(5) : button(4);
  }

  public Trigger b() {
    return Robot.isSimulation() ? button(2) : button(2);
  }

  public Trigger a() {
    return Robot.isSimulation() ? button(1) : button(1);
  }

  public Trigger x() {
    return Robot.isSimulation() ? button(4) : button(3);
  }

  public Trigger leftBumper() {
    return Robot.isSimulation() ? button(7) : button(5);
  }

  public Trigger rightBumper() {
    return Robot.isSimulation() ? button(8) : button(6);
  }
}
