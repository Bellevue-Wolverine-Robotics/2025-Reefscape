package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OperatorConstants;

public class Debug {
  public static final CommandXboxController driverXbox = new CommandXboxController(0);

  public static final void debugXboxController() {
    for (int i = 1; i <= 10; i++) {
      if (driverXbox.button(i).getAsBoolean()) {
        System.out.println("Button " + i + " is pressed");
      }
    }

    for (int i = 0; i < 8; i++) {
      // the triggers are constantly -1
      if (i == 4 || i == 5) {
        continue;
      }

      if (Math.abs(driverXbox.getRawAxis(i)) > OperatorConstants.kDeadzone) {
        System.out.println("Axis " + i + " is " + driverXbox.getRawAxis(i));
      }
    }
  }
}
