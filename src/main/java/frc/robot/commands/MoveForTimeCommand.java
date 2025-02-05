package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMotor;

import java.time.Duration;
import java.time.Instant;

public class MoveForTimeCommand extends Command {
    private final DriveSubsystem driveSubsystem;

    private final int movementTime;
    private final double rightMotorSpeed;
    private final double leftMotorSpeed;

    private boolean ended;
    private Instant starttime;

    // Dist per revolution = 18.85 inches
    // 5676 RPM = 94.6 rps

    /**
   * Creates a new MoveForTimeCommand
   *
   * @param subsystem The subsystem used by this command.
   * @param movementTime The time in milliseconds the motors will be moving at the given speed
   * @param rightMotorSpeed The speed to set the right motor at. Value should be between -1.0 and 1.0.
   * @param leftMotorSpeed The speed to set the left motor at. Value should be between -1.0 and 1.0.
   */
    public MoveForTimeCommand(DriveSubsystem subsystem, int movementTime, double rightMotorSpeed, double leftMotorSpeed) {
        driveSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        this.movementTime = movementTime;
        this.rightMotorSpeed = rightMotorSpeed;
        this.leftMotorSpeed = leftMotorSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ended = false;
        starttime = Instant.now();
        driveSubsystem.setMotor(DriveMotor.LEFTMOTOR, leftMotorSpeed);
        driveSubsystem.setMotor(DriveMotor.RIGHTMOTOR, rightMotorSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Feed the safety watchdog to affirm the motors should be running.
        driveSubsystem.safetyFeed();

        if (Duration.between(starttime, Instant.now()).toMillis() > movementTime)
        {
            ended = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopAllMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ended;
    }
}
