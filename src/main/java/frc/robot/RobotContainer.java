// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(ElevatorConstants.LEVEL_ONE)));
    controller.b().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(ElevatorConstants.LEVEL_TWO)));
    controller.y().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(ElevatorConstants.LEVEL_THREE)));
    controller.x().onTrue(Commands.runOnce(() -> elevatorSubsystem.setPosition(ElevatorConstants.LEVEL_FOUR)));
  }
}
