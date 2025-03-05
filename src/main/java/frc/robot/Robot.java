
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput; // Import for DigitalInput (Coral Sensor example)
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Import for SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDModeSubsystem;

public class Robot extends TimedRobot {
    private LEDModeSubsystem m_ledSubsystem; // LED Subsystem instance
    private DigitalInput coralSensor; // Example: Digital Input for Coral Sensor
    private RobotContainer robotContainer;

    // --- Variables for Simulated AprilTag Tracking ---
    // In a real robot, you would get this from your vision subsystem
    private boolean isAprilTagDetected = false;
    private double aprilTagToggleTimer = 0; // Timer for simulating AprilTag status changes

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        // --- Initialize Subsystems ---
        /* m_ledSubsystem = new LEDModeSubsystem(); // Create the LED Subsystem

        // --- Initialize Sensors ---
        // Initialize the digital input for the coral sensor (adjust port number if needed)
        coralSensor = new DigitalInput(0); // Assuming Coral sensor is on Digital Input port 0, CHANGE IF DIFFERENT

        // --- Initialize SmartDashboard ---
        SmartDashboard.putBoolean("Simulate AprilTag Tracked", isAprilTagDetected); // Button to simulate AprilTag */
    }

    @Override
    public void robotPeriodic() {
        /* // This function runs every robot cycle (approx. 20ms)

        // --- Get Sensor Readings ---
        // Read the value from the coral sensor (it returns true if NO object is detected, false if object is close)
        // We invert it here so that 'hasCoral' is TRUE when an object IS detected.
        boolean coralSensorValue = !coralSensor.get(); // Invert the sensor reading to match 'hasCoral' logic

        // --- Simulate AprilTag Tracking (for demonstration) ---
        // In a real robot, you would replace this with actual vision code
        aprilTagToggleTimer += 0.02; // Increment timer by 20ms (periodic loop time)
        if (aprilTagToggleTimer >= 5.0) { // Toggle AprilTag status every 5 seconds for demonstration
            isAprilTagDetected = !isAprilTagDetected; // Toggle the AprilTag status
            SmartDashboard.putBoolean("Simulate AprilTag Tracked", isAprilTagDetected); // Update SmartDashboard
            aprilTagToggleTimer = 0; // Reset the timer
        }
        isAprilTagDetected = SmartDashboard.getBoolean("Simulate AprilTag Tracked", false); // Get simulated status from SmartDashboard


        // --- Update LED Subsystem ---
        // Send the sensor readings to the LED subsystem to update the LED colors
        m_ledSubsystem.setHasCoral(coralSensorValue); // Pass the coral sensor value to the LED subsystem
        m_ledSubsystem.setIsAprilTagTracked(isAprilTagDetected); // Pass the AprilTag tracking status to LED subsystem

        // m_ledSubsystem.periodic(); // IMPORTANT:  Make sure to call the periodic method of the LED subsystem!
        */
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        // m_ledSubsystem.periodic(); // Still call periodic in disabled to make LEDs flash green when disabled
    }

    @Override
    public void autonomousInit() {
        // m_ledSubsystem.stopBlinking(); // Example: Stop blinking at the start of autonomous
    }

    @Override
    public void autonomousPeriodic() {
        // m_ledSubsystem.periodic(); // Call periodic in autonomous
    }

    @Override
    public void teleopInit() {
        // m_ledSubsystem.stopBlinking(); // Example: Stop blinking at the start of teleop
    }

    @Override
    public void teleopPeriodic() {
        // m_ledSubsystem.periodic(); // Call periodic in teleop
    }

    @Override
    public void testPeriodic() {
        // m_ledSubsystem.periodic(); // Call periodic in test mode
    }
}
