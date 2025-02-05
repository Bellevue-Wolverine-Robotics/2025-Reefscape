// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {

    private final SparkMax leftMotor = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(2, MotorType.kBrushless);

    private final XboxController controller = new XboxController(0);
    private final DifferentialDrive robotDrive;

    private SparkMax[] motors;

    public enum DriveMotor {
        LEFTMOTOR(0),
        RIGHTMOTOR(1);

        private final int index;
        private DriveMotor(int index) {
            this.index = index;
        }
    }

    private boolean moving = false;

    public DriveSubsystem() {
        rightMotor.setInverted(true);

        motors = new SparkMax[]{leftMotor, rightMotor};

        robotDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);
    }

    /**
     * Gets the given motor's encoder.
     * 
     * @param motor The motor whose encoder will be returned.
     */
    public RelativeEncoder getMotorEncoder(DriveMotor motor) {
        return motors[motor.index].getEncoder();
    }

    /**
     * Sets the given motor at the given speed.
     * 
     * @param motor The motor whose speed will be set to the given <speed>.
     * @param speed Scale of 0-1 of the speed the motors will go at.
     * Negative for backwards.
     */
    public void setMotor(DriveMotor motor, double speed) {
        moving = true;
        motors[motor.index].set(speed);
    }

    /**
     * Stops all the motors, and sets private variable
     * {@code moving} to false, causing the system to be drivable
     * using the controller again.
     */
    public void stopAllMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        moving = false;
    }

    public void arcadeDrive(double xSpeed, double zRotation)
    {
        if (moving) return;
        robotDrive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * Feeds the watchdog, affirming that the motors haven't gone haywire.
     */
    public void safetyFeed() {
        robotDrive.feed();
    }
}