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
    SwerveInputStream driveDirectAngle = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
        () -> driverXbox.getLeftY(),
        () -> driverXbox.getLeftX())
        .withControllerHeadingAxis(
            () -> -driverXbox.getRightX(),
            () -> -driverXbox.getRightY())
        .headingWhile(true)
        .allianceRelativeControl(true)
        .deadband(OperatorConstants.kDeadzone);

    SwerveInputStream driveAngularSpeed = SwerveInputStream.of(driveSubsystem.getSwerveDrive(),
        () -> driverXbox.getLeftY(),
        () -> driverXbox.getLeftX())
        .withControllerRotationAxis(() -> -driverXbox.getRightX())
        .deadband(OperatorConstants.kDeadzone);

    Command driveDirectAngleCommand = driveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveAngularSpeedCommand = driveSubsystem.driveFieldOriented(driveAngularSpeed);

    driveSubsystem.setDefaultCommand(driveAngularSpeedCommand);

    driverXbox.rightTrigger().onFalse(new InstantCommand(() -> {
      System.out.println("driveAngularSpeed");
      driveSubsystem.setDefaultCommand(driveAngularSpeedCommand);
    }, driveSubsystem));

    driverXbox.rightTrigger().onTrue(new InstantCommand(() -> {
      System.out.println("driveDirectAngle");
      driveSubsystem.setDefaultCommand(driveDirectAngleCommand);
    }, driveSubsystem));

    // driverXbox.b().whileTrue(
    // driveSubsystem.driveToPose(
    // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
  }

  // private void aimAtTarget() {
  // // Calculate drivetrain commands from Joystick values
  // double forward = -driverXbox.getLeftY() *
  // DriveConstants.kMaxSpeedMetersPerSec;
  // double strafe = -driverXbox.getLeftX() *
  // DriveConstants.kMaxSpeedMetersPerSec;
  // double turn = -driverXbox.getRightX() * DriveConstants.kMaxSpeedMetersPerSec;

  // // Read in relevant data from the Camera
  // boolean targetVisible = false;
  // dnuble targetYaw = 0.0;
  // var results = camera.getAllUnreadResults();
  // if (!results.isEmpty()) {
  // // Camera processed a new frame since last
  // // Get the last one in the list.
  // var result = results.get(results.size() - 1);
  // if (result.hasTargets()) {
  // // At least one AprilTag was seen by the camera
  // for (var target : result.getTargets()) {
  // if (target.getFiducialId() == 7) {
  // // Found Tag 7, record its information
  // targetYaw = target.getYaw();
  // targetVisible = true;
  // }
  // }
  // }
  // }

  // // Auto-align when requested
  // if (driverXbox.getAButton() && targetVisible) {
  // // Driver wants auto-alignment to tag 7
  // // And, tag 7 is in sight, so we can turn toward it.
  // // Override the driver's turn command with an automatic one that turns toward
  // // the tag.
  // turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
  // }

  // // Command drivetrain motors based on target speeds
  // drivetrain.drive(forward, strafe, turn);

  // // Put debug information to the dashboard
  // SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
  // }
}
