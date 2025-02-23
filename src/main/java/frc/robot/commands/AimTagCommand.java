package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.AprilTagStruct;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.XboxControllerWrapper;
import swervelib.SwerveInputStream;

public class AimTagCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private VisionSubsystem visionSubsystem;
  private XboxControllerWrapper xboxController;
  private int targetID;

  public AimTagCommand(VisionSubsystem visionSubsystem,
      SwerveSubsystem swerveSubsystem,
      XboxControllerWrapper xboxController,
      int targetID) {

    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.xboxController = xboxController;
    this.targetID = targetID;

    addRequirements(swerveSubsystem);
  }

  private SwerveInputStream aimAtTargetSpeeds(
      XboxControllerWrapper xboxController,
      int id) {
    DoubleSupplier turnSupplier = () -> {
      AprilTagStruct tag = visionSubsystem.getTargetID(id);
      System.out.println(tag.yaw);
      return tag.targetVisible
          ? -tag.yaw * VisionConstants.kVisionTurnkP
          : xboxController.getRightX();
    };

    return swerveSubsystem.driveAngularSpeed(xboxController).withControllerRotationAxis(
        turnSupplier);
  }

  @Override
  public void execute() {
    swerveSubsystem.getSwerveDrive().driveFieldOriented(aimAtTargetSpeeds(xboxController, targetID).get());
  }
}
