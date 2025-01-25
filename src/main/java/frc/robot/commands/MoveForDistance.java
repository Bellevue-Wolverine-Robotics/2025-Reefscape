package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.TankDriveSubsystem.DriveMotor;

public class MoveForDistance extends Command {
    private static final double POSITION_CONVERSION_FACTOR = 18.85d;

    private final TankDriveSubsystem tankDriveSubsystem;

    private boolean ended;

    private float distance;
    private float rightMotorSpeed;
    private float leftMotorSpeed;

    // Dist per revolution = 18.85 inches
    // 5676 RPM = 94.6 rps

    /**
   * Creates a new MoveForDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param distance The distance in inches that the motors will be moving at the given speed.
   * Motors will stop after they've gone the given distance.
   * @param rightMotorSpeed Scale of 0-1 of the speed the right motor will go at.
   * @param leftMotorSpeed Scale of 0-1 of the speed the left motor will go at.
   */
    public MoveForDistance(TankDriveSubsystem subsystem, float distance, float rightMotorSpeed, float leftMotorSpeed) {
        tankDriveSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        this.distance = distance;
        this.rightMotorSpeed = rightMotorSpeed;
        this.leftMotorSpeed = leftMotorSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        tankDriveSubsystem.getMotorEncoder(DriveMotor.LEFTMOTOR).setPosition(0.0d);
        tankDriveSubsystem.getMotorEncoder(DriveMotor.RIGHTMOTOR).setPosition(0.0d);
        
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

        // Calculate the total distance traveled (of the robot, not one wheel) 
        // by averaging each motor's distance.
        double distance_traveled = calculateDistanceTraveled(new DriveMotor[] {DriveMotor.LEFTMOTOR, DriveMotor.RIGHTMOTOR});
        if (distance_traveled >= distance)
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

    private double calculateDistanceTraveled(DriveMotor[] driveMotors) {
        double total_distance = 0.0d;
        for (DriveMotor driveMotor : driveMotors) {
            total_distance += tankDriveSubsystem.getMotorEncoder(driveMotor).getPosition() * POSITION_CONVERSION_FACTOR;
        }
        return total_distance / driveMotors.length;

    }
}
