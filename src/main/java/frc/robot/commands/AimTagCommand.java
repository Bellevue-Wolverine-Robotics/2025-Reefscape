package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.XboxControllerWrapper;

public class AimTagCommand extends Command {

  private SwerveSubsystem swerveSubsystem;
  private VisionSubsystem visionSubsystem;
  private CommandXboxController xboxController;
  private int targetID;

  public AimTagCommand(
    VisionSubsystem visionSubsystem,
    SwerveSubsystem swerveSubsystem,
    CommandXboxController xboxController,
    int targetID
  ) {
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.xboxController = xboxController;
    this.targetID = targetID;

    addRequirements(swerveSubsystem);
  }
}
