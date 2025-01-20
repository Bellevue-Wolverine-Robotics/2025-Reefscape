// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TankDriveSubsystem extends SubsystemBase {

    private final SparkMax m_leftFront = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax m_leftBack = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax m_rightFront = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax m_rightBack = new SparkMax(1, MotorType.kBrushless);
    private final XboxController c_controller = new XboxController(0);

    private final DifferentialDrive m_robotDrive1 =
        new DifferentialDrive(m_leftFront::set, m_rightFront::set);

    private final DifferentialDrive m_robotDrive2 =
        new DifferentialDrive(m_leftBack::set, m_rightBack::set);


    private boolean goingForward = false;

    /** Creates a new ExampleSubsystem. */
    public TankDriveSubsystem() {
        m_rightFront.setInverted(true);
        m_rightBack.setInverted(true);
    }

    /**
     * Sets all motors at a speed.
     * 
     * @param speed Scale of 0-1 of the speed the motors will go at.
     * Negative for backwards.
     */
    public void SetAllMotors(float speed) {
        goingForward = true;
        m_leftFront.set(-speed);
        m_leftBack.set(-speed);
        m_rightFront.set(speed);
        m_rightBack.set(speed);
    }

    public void StopAllMotors()
    {
        m_leftFront.stopMotor();
        m_leftBack.stopMotor();
        m_rightFront.stopMotor();
        m_rightBack.stopMotor();
        goingForward = false;
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        if (!goingForward)
        {
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

            m_robotDrive1.arcadeDrive(-c_controller.getLeftY(), -c_controller.getRightX());
            m_robotDrive2.arcadeDrive(-c_controller.getLeftY(), -c_controller.getRightX());
        }
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}