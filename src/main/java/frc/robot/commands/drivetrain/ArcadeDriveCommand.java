package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubystem;

public class ArcadeDriveCommand extends Command {
    private DriveSubystem m_driveSubsystem;
    private DoubleSupplier m_xSpeedSupplier;
    private DoubleSupplier m_zRotSupplier;

    public ArcadeDriveCommand(DriveSubystem drivesystem, DoubleSupplier xSpeedSupplier, DoubleSupplier zRotSupplier) {
        this.m_driveSubsystem = drivesystem;

        m_xSpeedSupplier = xSpeedSupplier;
        m_zRotSupplier = zRotSupplier;

        //addRequirements(drivesystem);
    }

    @Override
    public void execute() {
        // System.out.println("xSpeed: " + m_xSpeedSupplier.getAsDouble());
        // System.out.println("zRot: " + m_zRotSupplier.getAsDouble());
        //m_driveSubsystem.arcadeDrive(m_xSpeedSupplier.getAsDouble(), m_zRotSupplier.getAsDouble());
    }
}