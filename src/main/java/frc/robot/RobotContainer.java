package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER_ID);
    private final CommandXboxController elevatorController = new CommandXboxController(OperatorConstants.ELEVATOR_CONTROLLER_ID);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        driverController.x().onTrue(Commands.runOnce(() -> elevatorSubsystem.adjust()));

        elevatorController.x().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_ONE)));
        elevatorController.y().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_TWO)));
        elevatorController.b().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_THREE)));
        elevatorController.a().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.LEVEL_FOUR)));
        elevatorController.leftBumper().onTrue(Commands.runOnce(() -> elevatorSubsystem.setTarget(ElevatorConstants.INTAKE_LEVEL)));

        elevatorController.leftTrigger().onTrue(Commands.runOnce(() -> coralSubsystem.unjam(true)));
        elevatorController.leftTrigger().onFalse(Commands.runOnce(() -> coralSubsystem.unjam(false)));
        elevatorController.rightTrigger().onTrue(Commands.runOnce(() -> coralSubsystem.eject(true)));
        elevatorController.rightTrigger().onFalse(Commands.runOnce(() -> coralSubsystem.eject(true)));
    }
}
