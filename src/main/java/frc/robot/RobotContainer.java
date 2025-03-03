package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimTagCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.TriggerUtil;
import frc.robot.utils.XboxControllerWrapper;
import java.io.File;
import java.util.function.Supplier;

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
    setDefaultPathfinding(
      driverXbox.y(),
      driverXbox.b(),
      driverXbox.a(),
      driverXbox.x(),
      driverXbox.rightBumper(),
      driverXbox.leftBumper()
    );
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

    Command slowDriveCommand = driveSubsystem.driveAngularSpeedCommand(
      driverXbox,
      DriveConstants.SLOW_COEF
    );

    TriggerUtil.holdChangeDefault(
      driverXbox.rightTrigger(),
      driveSubsystem,
      driveAngularSpeedCommand,
      slowDriveCommand
    );

    driveSubsystem.setDefaultCommand(driveAngularSpeedCommand);
    // TriggerUtil.holdChangeDefault(driverXbox.rightTrigger(), driveSubsystem,
    // driveAngularSpeedCommand,
    // driveDirectAngleCommand);

    //     TriggerUtil.holdChangeDefault(
    //       driverXbox.b(),
    //       driveSubsystem,
    //       driveAngularSpeedCommand,
    //       aimAtTargetCommand
    //     );

    //     TriggerUtil.holdChangeDefault(
    //       driverXbox.a(),
    //       driveSubsystem,
    //       driveAngularSpeedCommand,
    //       chaseTagCommand
    //     );
  }

  private void setDefaultPathfinding(
    Trigger top,
    Trigger right,
    Trigger bottom,
    Trigger left,
    Trigger rightBumper,
    Trigger leftBumper
  ) {
    Trigger bottomOnly = bottom.and(left.negate()).and(right.negate());
    Trigger topOnly = top.and(left.negate()).and(right.negate());

    // Map solo triggers to their corresponding poses
    setupPathfindingTrigger(
      bottomOnly,
      AprilTagConstants.getBottomTagApproachPoseSupplier()
    );
    setupPathfindingTrigger(
      topOnly,
      AprilTagConstants.getTopTagApproachPoseSupplier()
    );

    // Coral station triggers remain unchanged
    setupPathfindingTrigger(
      rightBumper,
      AprilTagConstants.getRightCoralStationApproachPoseSupplier()
    );
    setupPathfindingTrigger(
      leftBumper,
      AprilTagConstants.getLeftCoralStationApproachPoseSupplier()
    );

    // Compound triggers remain the same
    setupPathfindingTrigger(
      bottom.and(left),
      AprilTagConstants.getBottomLeftTagApproachPoseSupplier()
    );
    setupPathfindingTrigger(
      bottom.and(right),
      AprilTagConstants.getBottomRightTagApproachPoseSupplier()
    );
    setupPathfindingTrigger(
      top.and(left),
      AprilTagConstants.getTopLeftTagApproachPoseSupplier()
    );
    setupPathfindingTrigger(
      top.and(right),
      AprilTagConstants.getTopRightTagApproachPoseSupplier()
    );
  }

  /**
   * Helper method to set up a pathfinding command for a trigger using a pose supplier
   * @param trigger The trigger that activates the command
   * @param poseSupplier The supplier that provides the target pose when needed
   */
  private void setupPathfindingTrigger(
    Trigger trigger,
    Supplier<Pose2d> poseSupplier
  ) {
    trigger.whileTrue(driveSubsystem.driveToPose(poseSupplier));
  }
}
