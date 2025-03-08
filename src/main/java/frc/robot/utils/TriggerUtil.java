package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerUtil {
  public static void holdChangeDefault(Trigger trigger, Subsystem subsystem, Command offCommand, Command onCommand) {
    trigger.onFalse(new InstantCommand(() -> {
      subsystem.setDefaultCommand(offCommand);
    }, subsystem));

    trigger.onTrue(new InstantCommand(() -> {
      subsystem.setDefaultCommand(onCommand);
    }, subsystem));
  }
}
