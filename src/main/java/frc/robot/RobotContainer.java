// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.MoveForDistanceCommand;
import frc.robot.commands.MoveForTimeCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final MoveForTimeCommand goForwardForTwoHalfSecondCommand = new MoveForTimeCommand(driveSubsystem, 2500, 0.25d, 0.25d);
  private final MoveForDistanceCommand goForwardHundredInches = new MoveForDistanceCommand(driveSubsystem, 10.0d, 0.050d, 0.050d);
  
  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final XboxController xboxControllerHID = xboxController.getHID();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    CommandScheduler.getInstance().registerSubsystem(driveSubsystem);
    driveSubsystem.setDefaultCommand(new RunCommand(() -> 
      driveSubsystem.arcadeDrive(xboxControllerHID.getLeftY() / 3.0d, xboxControllerHID.getLeftX() / 3.0d), driveSubsystem));
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule command on b button pressed.
    xboxController.b().onTrue(goForwardForTwoHalfSecondCommand);
    xboxController.x().onTrue(goForwardHundredInches);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  /*
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  */
}
