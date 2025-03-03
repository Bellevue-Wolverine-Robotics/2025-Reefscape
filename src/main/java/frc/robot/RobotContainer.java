package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AimTagCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.AprilTagUtils;
import frc.robot.utils.TriggerUtil;
import frc.robot.utils.XboxControllerWrapper;
import java.io.File;

/**
 * Where most of the structure of the robot, including subsystems, commands, and
 * button mappings are declared.
 */
public class RobotContainer {

  final XboxControllerWrapper driverXbox = new XboxControllerWrapper(
    0,
    false,
    false
  );

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve"),
    visionSubsystem
  );

  // private final VisionSubsystem visionSubsystem = new VisionSim(() ->
  //   driveSubsystem.getPose()
  // );

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
    Command driveDirectAngleCommand = driveSubsystem.driveDirectAngleCommand(
      driverXbox
    );
    Command driveAngularSpeedCommand = driveSubsystem.driveAngularSpeedCommand(
      driverXbox
    );

    // Command aimAtTargetCommand = driveSubsystem.aimAtTargetCommand(
    // driverXbox,
    // 18);

    Command aimAtTargetCommand = new AimTagCommand(
      visionSubsystem,
      driveSubsystem,
      driverXbox,
      18
    );
    Command chaseTagCommand = new ChaseTagCommand(
      visionSubsystem,
      driveSubsystem,
      1
    );

    driveSubsystem.setDefaultCommand(driveAngularSpeedCommand);

    // TriggerUtil.holdChangeDefault(driverXbox.rightTrigger(), driveSubsystem,
    // driveAngularSpeedCommand,
    // driveDirectAngleCommand);

    TriggerUtil.holdChangeDefault(
      driverXbox.b(),
      driveSubsystem,
      driveAngularSpeedCommand,
      aimAtTargetCommand
    );

    TriggerUtil.holdChangeDefault(
      driverXbox.a(),
      driveSubsystem,
      driveAngularSpeedCommand,
      chaseTagCommand
    );

    driverXbox
      .a()
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(18).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI))
          )
        )
      );

    driverXbox
      .y()
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(21).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI))
          )
        )
      );

    driverXbox
      .a()
      .and(driverXbox.x())
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(19).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI))
          )
        )
      );

    driverXbox
      .x()
      .and(driverXbox.y())
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(20).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI))
          )
        )
      );

    driverXbox
      .b()
      .and(driverXbox.y())
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(22).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI))
          )
        )
      );

    driverXbox
      .b()
      .and(driverXbox.a())
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(17).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(Math.PI))
          )
        )
      );
    driverXbox
      .rightBumper()
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(12).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(0))
          )
        )
      );

    driverXbox
      .leftBumper()
      .whileTrue(
        driveSubsystem.driveToPose(
          AprilTagUtils.getAprilTagPose(13).transformBy(
            new Transform2d(new Translation2d(1, 0), new Rotation2d(0))
          )
        )
      );
  }
}
