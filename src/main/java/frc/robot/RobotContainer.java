package frc.robot;

import java.io.File;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.TriggerUtil;
import frc.robot.utils.XboxControllerWrapper;
import swervelib.SwerveInputStream;
import frc.robot.constants.*;

/**
 * Where most of the structure of the robot, including subsystems, commands, and
 * button mappings are declared.
 */
public class RobotContainer {
  final XboxControllerWrapper driverXbox = new XboxControllerWrapper(0);
  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    setDefaultDriveBehavior();
    // driverXbox.rightBumper().whileTrue(
    // driveAngularSpeedCommand
    // .andThen(() -> {
    // System.out.println("Overriding default command with
    // driveAngularSpeedCommand");
    // driveSubsystem.setDefaultCommand(driveAngularSpeedCommand);
    // }, driveSubsystem)
    // .finallyDo((interrupted) -> {
    // System.out.println("Restoring default command");
    // driveSubsystem.setDefaultCommand(driveDirectAngleCommand);
    // }));

    // if (DriverStation.isTest()) {
    // driverXbox.x().whileTrue(Commands.runOnce(driveSubsystem::lock,
    // driveSubsystem).repeatedly());
    // driverXbox.y().whileTrue(driveSubsystem.driveToDistanceCommand(1.0, 0.2));
    // driverXbox.start().onTrue((Commands.runOnce(driveSubsystem::zeroGyro)));
    // driverXbox.back().whileTrue(driveSubsystem.centerModulesCommand());
    // driverXbox.leftBumper().onTrue(Commands.none());
    // driverXbox.rightBumper().onTrue(Commands.none());
    // } else {
    // driverXbox.a().onTrue((Commands.runOnce(driveSubsystem::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(driveSubsystem::addFakeVisionReading));
    // driverXbox.b().whileTrue(
    // driveSubsystem.driveToPose(
    // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
    // driverXbox.start().whileTrue(Commands.none());
    // driverXbox.back().whileTrue(Commands.none());
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(driveSubsystem::lock,
    // driveSubsystem).repeatedly());
    // driverXbox.rightBumper().onTrue(Commands.none());
    // }
  }

  private void setDefaultDriveBehavior() {
    Command driveDirectAngleCommand = driveSubsystem.driveDirectAngleCommand(driverXbox);
    Command driveAngularSpeedCommand = driveSubsystem.driveAngularSpeedCommand(driverXbox);

    Command aimAtTarget = driveSubsystem.aimAtTarget(driverXbox, visionSubsystem, 1);

    driveSubsystem.setDefaultCommand(driveAngularSpeedCommand);

    // TriggerUtil.holdChangeDefault(driverXbox.rightTrigger(), driveSubsystem,
    //     driveAngularSpeedCommand,
    //     driveDirectAngleCommand);

    // TriggerUtil.holdCHangeDefault(driverXbox.b(), driveSubsystem,
    // driveAngularSpeedCommand, aimAtTarget);

    TriggerUtil.holdChangeDefault(driverXbox.b(), driveSubsystem,
        driveAngularSpeedCommand,
        aimAtTarget);

    // driverXbox.b().whileTrue(
    // driveSubsystem.driveToPose(
    // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
  }
}
