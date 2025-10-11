package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.File;
import java.util.function.Supplier;

public class RobotContainer {
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), visionSubsystem);

  private final CommandXboxController driverController = new CommandXboxController(DriveConstants.CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.CONTROLLER_PORT);
  
  public RobotContainer() {
    setDriverBindings();
    setOperatorBindings();
    setPathfindingBindings(
      driverController.y(),
      driverController.b(),
      driverController.a(),
      driverController.x(),
      driverController.rightBumper(),
      driverController.leftBumper()
    );
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void setDriverBindings() {
    driverController.start().onTrue((Commands.runOnce(driveSubsystem::zeroGyro)));
    driverController.back().whileTrue(driveSubsystem.centerModulesCommand());
    driverController.rightTrigger().whileTrue(driveSubsystem.driveAngularSpeedCommand(driverController, DriveConstants.SLOW_COEF));  
    driveSubsystem.setDefaultCommand(driveSubsystem.driveAngularSpeedCommand(driverController));
  }

  private void setOperatorBindings() {
    operatorController.x().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_TWO));
    operatorController.y().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_THREE));
    operatorController.b().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_FOUR));
    operatorController.a().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_ZERO));
    operatorController.leftBumper().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_ONE));
    operatorController.axisMagnitudeGreaterThan(1, OperatorConstants.LEFT_Y_DEADBAND).whileTrue(elevatorSubsystem.moveManual(() -> operatorController.getLeftY()) );
    operatorController.leftTrigger().whileTrue(coralSubsystem.unjam());
    operatorController.rightTrigger().whileTrue(coralSubsystem.eject());
  }

  private void setPathfindingBindings(
    Trigger top,
    Trigger right,
    Trigger bottom,
    Trigger left,
    Trigger rightBumper,
    Trigger leftBumper
  ) {
    Trigger bottomOnly = bottom.and(left.negate()).and(right.negate());
    Trigger topOnly = top.and(left.negate()).and(right.negate());

    setupPathfindingTrigger(bottomOnly, AprilTagConstants.getBottomTagApproachPoseSupplier());
    setupPathfindingTrigger(topOnly, AprilTagConstants.getTopTagApproachPoseSupplier());
    setupPathfindingTrigger(rightBumper, AprilTagConstants.getRightCoralStationApproachPoseSupplier());
    setupPathfindingTrigger(leftBumper, AprilTagConstants.getLeftCoralStationApproachPoseSupplier());
    setupPathfindingTrigger(bottom.and(left), AprilTagConstants.getBottomLeftTagApproachPoseSupplier());
    setupPathfindingTrigger(bottom.and(right), AprilTagConstants.getBottomRightTagApproachPoseSupplier());
    setupPathfindingTrigger(top.and(left), AprilTagConstants.getTopLeftTagApproachPoseSupplier());
    setupPathfindingTrigger(top.and(right), AprilTagConstants.getTopRightTagApproachPoseSupplier());
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

  public Command getAutonomousCommand() {
    return Commands.runOnce(driveSubsystem::zeroGyro).andThen(driveSubsystem.driveCommand(() -> -0.5, () -> 0, () -> 0));
  }
}
