package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.DriverStationConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  @SuppressWarnings("unused")
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  @SuppressWarnings("unused")
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(DriverStationConstants.OPERATOR_CONTROLLER_PORT);
  
  public RobotContainer() {
    setDriverBindings();
    setOperatorBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void setDriverBindings() {
    driverController.start().onTrue(swerveSubsystem.zeroGyro());
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX()
    ));

    driverController.x().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.LEFT_BRANCH_SUPPLIER));
    driverController.b().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.RIGHT_BRANCH_SUPPLIER));
  }

  private void setOperatorBindings() {
    operatorController.x().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_TWO));
    operatorController.y().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_THREE));
    operatorController.b().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_FOUR));
    operatorController.a().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_ZERO));
    operatorController.leftBumper().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_ONE));
    operatorController.axisMagnitudeGreaterThan(1, DriverStationConstants.OPERATOR_CONTROLLER_LEFT_DEADBAND).whileTrue(elevatorSubsystem.moveManual(() -> operatorController.getLeftY()) );
    operatorController.leftTrigger().whileTrue(coralSubsystem.unjam());
    operatorController.rightTrigger().whileTrue(coralSubsystem.eject());
    operatorController.start().onTrue(elevatorSubsystem.reset());
  }

  public Command getAutonomousCommand() {
    return Commands.runOnce(swerveSubsystem::zeroGyro);
  }
}
