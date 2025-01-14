package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.constants.IOConstants.DriverButtonConstants;
import frc.robot.constants.IOConstants.OperatorButtonConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.commands.FullRoutines;

import frc.robot.commands.drivetrain.ArcadeDriveCommand;

public class RobotContainer {

    // Subsystems
    private final DriveSubystem m_driveSubsystem = new DriveSubystem();
    // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    // private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

    // Joysticks and Buttons
    private final Joystick m_driverJoystick = new Joystick(0);
    private final Joystick m_operatorJoystick = new Joystick(1);

    // Constructor
    public RobotContainer() {
        // Set up default commands
        //m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, m_driverJoystick));

        // Configure button bindings
        configureButtonBindings();

      // Add options to the throttle selection
       //throttleSelection.addOption("Default Throttle", ThrottlesSmartdashboard.DEFAULT);
       // throttleSelection.addOption("Alternate Throttle", ThrottlesSmartdashboard.ALTERNATE);
    }

    // Configure button bindings
    private void configureButtonBindings() {
        // Driver buttons
        new JoystickButton(m_driverJoystick, DriverButtonConstants.kDriveSpeedPreset1Button)
            .whenPressed(new FullRoutines(m_driveSubsystem, m_flywheelSubsystem));

        // Operator buttons
        new JoystickButton(m_operatorJoystick, OperatorButtonConstants.kClimbUpButton)
            .whenPressed(new ClimberExtendCommand(m_climberSubsystem));

        new JoystickButton(m_operatorJoystick, OperatorButtonConstants.kClimbDownButton)
            .whenPressed(new ClimberRetractCommand(m_climberSubsystem));

        new JoystickButton(m_operatorJoystick, OperatorButtonConstants.kIntakeDeployButton)
            .whenPressed(new DeployIntakeCommand(m_intakeSubsystem));

        new JoystickButton(m_operatorJoystick, OperatorButtonConstants.kIntakeStowButton)
            .whenPressed(new StowIntakeCommand(m_intakeSubsystem));

        // Flywheel commands
        new JoystickButton(m_operatorJoystick, OperatorButtonConstants.kFlywheelAimButton)
            .whenPressed(new FlywheelAimIntakeReceiveCommand(m_flywheelSubsystem));

        new JoystickButton(m_operatorJoystick, OperatorButtonConstants.kFlywheelCalibrateButton)
            .whenPressed(new FlywheelCalibrate(m_flywheelSubsystem));

        new JoystickButton(m_operatorJoystick, OperatorButtonConstants.kFlywheelClimbModeButton)
            .whenPressed(new FlywheelClimbModeCommand(m_flywheelSubsystem));
    }

    // Get the autonomous command
    //public Command getAutonomousCommand() {
        // Placeholder: return your autonomous command here
        //return new FullRoutines(m_driveSubsystem, m_flywheelSubsystem);
    //}

    // Getters for subsystems
    public DriveSubystem getDriveSubsystem() {
        return m_driveSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return m_intakeSubsystem;
    }

    public FlywheelSubsystem getFlywheelSubsystem() {
        return m_flywheelSubsystem;
    }

   //public ClimberSubsystem getClimberSubsystem() {
  //    return m_climberSubsystem;
   //}

    // Get throttle selection from SmartDashboard
    //public ThrottlesSmartdashboard getThrottleSelection() {
        //return throttleSelection.getSelected();
    //}

    // Update the previous throttle setting (e.g., for diagnostics or logging)
    //public void updatePrevThrottle() {
        //prevThrottle = getThrottleSelection();
    //}
}