package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.DriveIO;
import frc.robot.interfaces.DriveIOInputsAutoLogged;
import frc.robot.interfaces.GyroIO;
import frc.robot.interfaces.GyroIOInputsAutoLogged;

/**
 * Class for the tank drivetrain subsystem
 */
public class DriveSubsystem extends SubsystemBase {
  private final DriveIO driveIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
}
