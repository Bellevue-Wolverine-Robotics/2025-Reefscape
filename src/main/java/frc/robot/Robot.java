// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {



  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*
    if (c_controller.getAButtonPressed()) speed = 0.075;
    else if (c_controller.getBButtonPressed()) speed = 0.10;
    else if (c_controller.getXButtonPressed()) speed = 0.05;
    else if (c_controller.getYButtonPressed()) speed = 0.125;
    else if (c_controller.getRawButtonPressed(8)) {
      if (drive_mode + 1 > max_drive_mode) drive_mode = 0;
      else drive_mode++;
    }

    if (drive_mode == 0)
    {
      double left_stick_vertical_axis = c_controller.getRawAxis(1);
      double right_stick_vertical_axis = c_controller.getRawAxis(5);
      
      m_leftFront.set(-left_stick_vertical_axis * speed);
      m_leftBack.set(-left_stick_vertical_axis * speed);
      m_rightFront.set(right_stick_vertical_axis * speed);
      m_rightBack.set(right_stick_vertical_axis * speed);
    }

    else if (drive_mode == 1)
    {
      double left_stick_horizontal_axis = c_controller.getRawAxis(0);
      double rotation_coeffecient = 2 * left_stick_horizontal_axis + 1;
      double right_trigger_axis = c_controller.getRawAxis(3);

      m_leftFront.set(-right_trigger_axis * -rotation_coeffecient * speed);
      m_leftBack.set(-right_trigger_axis * -rotation_coeffecient * speed);
      m_rightFront.set(right_trigger_axis * rotation_coeffecient * speed);
      m_rightBack.set(right_trigger_axis * rotation_coeffecient * speed);




    }
  }
  */}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
