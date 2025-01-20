package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveSubsystem;

import java.time.Duration;
import java.time.Instant;

public class GoForwardForOneSecondCommand extends Command {
    private final TankDriveSubsystem tankDriveSubsystem;

    private boolean ended;
    private Instant starttime;

    /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public GoForwardForOneSecondCommand(TankDriveSubsystem subsystem) {
        tankDriveSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ended = false;
        starttime = Instant.now();
        tankDriveSubsystem.SetAllMotors(0.066f);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Duration.between(starttime, Instant.now()).toMillis() > 1000)
        {
            ended = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        tankDriveSubsystem.StopAllMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ended;
    }
}
