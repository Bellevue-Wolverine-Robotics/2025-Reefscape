package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final CoralSubsystem coralSubsystem;
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        coralSubsystem = new CoralSubsystem();
        configureBindings();

        coralSubsystem.register();
         elevatorSubsystem.register();
    }

    private void configureBindings() {
        driverController.leftTrigger().whileTrue(elevatorSubsystem.holdScoringLevel());
    
        operatorController.x().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_ONE));
        operatorController.y().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_TWO));
        operatorController.b().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_THREE));
        operatorController.a().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.LEVEL_FOUR));
        operatorController.leftBumper().onTrue(elevatorSubsystem.setScoringPosition(ElevatorConstants.BOTTOM_LEVEL));

        operatorController.leftTrigger().whileTrue(coralSubsystem.unjam());
        operatorController.rightTrigger().whileTrue(coralSubsystem.eject());
    }
}
