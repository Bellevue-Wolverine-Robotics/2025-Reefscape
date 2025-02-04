// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.io.Console;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TankDriveSubsystem extends SubsystemBase {

    private final SparkMax leftMotor = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(2, MotorType.kBrushless);
    private final XboxController c_controller = new XboxController(0);

    private final DifferentialDrive robotDrive =
        new DifferentialDrive(leftMotor::set, rightMotor::set);

    private SparkMax[] motors = {leftMotor, rightMotor};

    public enum DriveMotor {
        LEFTMOTOR(0),
        RIGHTMOTOR(1);

        private final int index;
        private DriveMotor(int index) {
            this.index = index;
        }
    }

    private boolean moving = false;

    public TankDriveSubsystem() {
        // I know its deprecated but too bad so sad
        rightMotor.setInverted(true);
        robotDrive.setSafetyEnabled(false);
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
     * <moving> to false, causing the system to be drivable
     * using the controller again.
     */
    public void stopAllMotors() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        moving = false;
    }

    @Override
    public void periodic() {
        if (!moving) {
            /*
            // This method will be called once per scheduler run
            double left_stick_vertical_axis = c_controller.getRawAxis(1);
            double right_stick_vertical_axis = c_controller.getRawAxis(5);
            if (left_stick_vertical_axis > 0.5d) System.out.print("  i  ");
            
            m_leftFront.set(-left_stick_vertical_axis * 0.125);
            m_leftBack.set(-left_stick_vertical_axis * 0.125);
            m_rightFront.set(right_stick_vertical_axis * 0.125);
            m_rightBack.set(right_stick_vertical_axis * 0.125);
            */

            //robotDrive.arcadeDrive(-c_controller.getLeftY()/9.0d, -c_controller.getLeftX()/9.0d);
            //rightMotor.set(0.1d);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}