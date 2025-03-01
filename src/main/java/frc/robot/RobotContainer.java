// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER_ID);
  private final CommandXboxController elevatorController = new CommandXboxController(OperatorConstants.ELEVATOR_CONTROLLER_ID);

  public RobotContainer() {
      configureBindings();
  }

  private void configureBindings() {
      elevatorController.a().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_ONE)));
      elevatorController.b().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_TWO)));
      elevatorController.y().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_THREE)));
      elevatorController.x().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_FOUR)));
      driverController.x().onTrue(Commands.runOnce(() -> elevatorSubsystem.adjust()));
  }
}
