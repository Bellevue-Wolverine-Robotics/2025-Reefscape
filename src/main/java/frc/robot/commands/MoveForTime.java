package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.TankDriveSubsystem.DriveMotor;

import java.time.Duration;
import java.time.Instant;

public class MoveForTime extends Command {
    private final TankDriveSubsystem tankDriveSubsystem;

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
    public MoveForTime(TankDriveSubsystem subsystem, int movementTime, double rightMotorSpeed, double leftMotorSpeed) {
        tankDriveSubsystem = subsystem;
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
        tankDriveSubsystem.setMotor(DriveMotor.LEFTMOTOR, leftMotorSpeed);
        tankDriveSubsystem.setMotor(DriveMotor.RIGHTMOTOR, rightMotorSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Setting the speed repeatedly makes the safety watchdog happy,
        // no other reason. Without this, it shuts down in 100ms.
        tankDriveSubsystem.setMotor(DriveMotor.LEFTMOTOR, leftMotorSpeed);
        tankDriveSubsystem.setMotor(DriveMotor.RIGHTMOTOR, rightMotorSpeed);
        if (Duration.between(starttime, Instant.now()).toMillis() > movementTime)
        {
            ended = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        tankDriveSubsystem.stopAllMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ended;
    }
}
