package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.geometry.Pose2d;

import java.io.File;
import java.util.function.Supplier;

import frc.robot.commands.AimTagCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.utils.TriggerUtil;
import frc.robot.utils.XboxControllerWrapper;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDModeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Where most of the structure of the robot, including subsystems, commands, and
 * button mappings are declared.
 */
public class RobotContainer {

  // final XboxControllerWrapper driverController = new XboxControllerWrapper(
  //   0,
  //   false,
  //   false
  // );

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve"),
    visionSubsystem
  );

  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final LEDModeSubsystem ledSubsystem = new LEDModeSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

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

    coralSubsystem.register();
    elevatorSubsystem.register();
    ledSubsystem.register();
  }

  private void configureBindings() {
    setDefaultDriveBinds();
    setDefaultPathfinding(
      driverController.y(),
      driverController.b(),
      driverController.a(),
      driverController.x(),
      driverController.rightBumper(),
      driverController.leftBumper()
    );
    setDefaultOperatorBinds();

    // driverController.rightBumper().whileTrue(
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
    // driverController.x().whileTrue(Commands.runOnce(driveSubsystem::lock,
    // driveSubsystem).repeatedly());
    // driverController.y().whileTrue(driveSubsystem.driveToDistanceCommand(1.0, 0.2));
    // driverController.start().onTrue((Commands.runOnce(driveSubsystem::zeroGyro)));
    // driverController.back().whileTrue(driveSubsystem.centerModulesCommand());
    // driverController.leftBumper().onTrue(Commands.none());
    // driverController.rightBumper().onTrue(Commands.none());
    // } else {
    // driverController.a().onTrue((Commands.runOnce(driveSubsystem::zeroGyro)));
    // driverController.x().onTrue(Commands.runOnce(driveSubsystem::addFakeVisionReading));
    // driverController.b().whileTrue(
    // driveSubsystem.driveToPose(
    // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
    // driverController.start().whileTrue(Commands.none());
    // driverController.back().whileTrue(Commands.none());
    // driverController.leftBumper().whileTrue(Commands.runOnce(driveSubsystem::lock,
    // driveSubsystem).repeatedly());
    // driverController.rightBumper().onTrue(Commands.none());
    // }
  }

  private void setDefaultDriveBinds() {
    Command driveDirectAngleCommand = driveSubsystem.driveDirectAngleCommand(
      driverController
    );
    Command driveAngularSpeedCommand = driveSubsystem.driveAngularSpeedCommand(
      driverController
    );

    // Command aimAtTargetCommand = driveSubsystem.aimAtTargetCommand(
    // driverController,
    // 18);

    Command aimAtTargetCommand = new AimTagCommand(
      visionSubsystem,
      driveSubsystem,
      driverController,
      18
    );
    Command chaseTagCommand = new ChaseTagCommand(
      visionSubsystem,
      driveSubsystem,
      1
    );

    Command slowDriveCommand = driveSubsystem.driveAngularSpeedCommand(
      driverController,
      DriveConstants.SLOW_COEF
    );

    TriggerUtil.holdChangeDefault(
      driverController.rightTrigger(),
      driveSubsystem,
      driveAngularSpeedCommand,
      slowDriveCommand
    );

    driveSubsystem.setDefaultCommand(driveAngularSpeedCommand);
    // TriggerUtil.holdChangeDefault(driverController.rightTrigger(), driveSubsystem,
    // driveAngularSpeedCommand,
    // driveDirectAngleCommand);

    //     TriggerUtil.holdChangeDefault(
    //       driverController.b(),
    //       driveSubsystem,
    //       driveAngularSpeedCommand,
    //       aimAtTargetCommand
    //     );

    //     TriggerUtil.holdChangeDefault(
    //       driverController.a(),
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

  private void setDefaultOperatorBinds() {
    if (OperatorConstants.CONTROL_MODE == OperatorConstants.ControlMode.PARTIAL_OPERATOR) {
      driverController.leftTrigger().whileTrue(elevatorSubsystem.holdScoringPosition());
    }

    operatorController.x().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_TWO));
    operatorController.y().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_THREE));
    operatorController.b().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_FOUR));
    operatorController.a().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_ONE));
    operatorController.leftTrigger().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_ZERO));
    operatorController.rightTrigger().whileTrue(coralSubsystem.eject());
    operatorController.leftBumper().whileTrue(coralSubsystem.unjam());

    new POVButton(operatorController.getHID(), 0).whileTrue(elevatorSubsystem.moveUp());
    new POVButton(operatorController.getHID(), 180).whileTrue(elevatorSubsystem.moveDown());

  }
}
