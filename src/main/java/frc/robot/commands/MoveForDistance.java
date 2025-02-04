package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.TankDriveSubsystem.DriveMotor;

public class MoveForDistance extends Command {
    //private static final double POSITION_CONVERSION_FACTOR = 18.85d;
    private static final double POSITION_CONVERSION_FACTOR = 1.0d;

    private final TankDriveSubsystem tankDriveSubsystem;

    private final RelativeEncoder rightMotorEncoder;
    private final RelativeEncoder leftMotorEncoder;

    private final double distance;
    private final double rightMotorSpeed;
    private final double leftMotorSpeed;

    private boolean ended;

    

    // Dist per revolution = 18.85 inches
    // 5676 RPM = 94.6 rps

    /**
   * Creates a new MoveForDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param distance The distance in inches that the motors will be moving at the given speed.
   * Motors will stop after they've gone the given distance.
   * @param rightMotorSpeed The speed to set the right motor at. Value should be between -1.0 and 1.0.
   * @param leftMotorSpeed The speed to set the left motor at. Value should be between -1.0 and 1.0.
   */
    public MoveForDistance(TankDriveSubsystem subsystem, double distance, double rightMotorSpeed, double leftMotorSpeed) {
        tankDriveSubsystem = subsystem;

        rightMotorEncoder = subsystem.getMotorEncoder(DriveMotor.RIGHTMOTOR);
        leftMotorEncoder = subsystem.getMotorEncoder(DriveMotor.LEFTMOTOR);

        addRequirements(subsystem);

        this.distance = distance;
        this.rightMotorSpeed = rightMotorSpeed;
        this.leftMotorSpeed = leftMotorSpeed;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        ended = false;
        rightMotorEncoder.setPosition(0.0d);
        leftMotorEncoder.setPosition(0.0d);

        tankDriveSubsystem.setMotor(DriveMotor.RIGHTMOTOR, rightMotorSpeed);
        tankDriveSubsystem.setMotor(DriveMotor.LEFTMOTOR, leftMotorSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Setting the speed repeatedly makes the safety watchdog happy,
        // no other reason. Without this, it shuts down in 100ms.
        tankDriveSubsystem.setMotor(DriveMotor.RIGHTMOTOR, rightMotorSpeed);
        tankDriveSubsystem.setMotor(DriveMotor.LEFTMOTOR, leftMotorSpeed);

        // Calculate the total distance traveled (of the robot, not one wheel) 
        // by averaging each motor's distance.
        double distance_traveled = calculateDistanceTraveled(new DriveMotor[] {DriveMotor.RIGHTMOTOR, DriveMotor.LEFTMOTOR});
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
        /*
        double total_distance = 0.0d;
        for (DriveMotor driveMotor : driveMotors) {
            total_distance += tankDriveSubsystem.getMotorEncoder(driveMotor).getPosition() * POSITION_CONVERSION_FACTOR;
        }
        return total_distance / driveMotors.length;
        */
        
        return leftMotorEncoder.getPosition();
    }
}
