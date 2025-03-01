package frc.robot.Subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.OIConstants;
//import frc.robot.Constants.PortConstants;
public class LEDModeSubsystem extends SubsystemBase{
  public void periodic() {

  }
  //////
/*
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;

public class LEDModeSubsystem extends SubsystemBase {
  boolean cubeMode = false;
  AddressableLED led;
  AddressableLEDBuffer ledBufferOff;
  Gripper gripper;
  AddressableLEDBuffer yellowLedBuffer;
  AddressableLEDBuffer purpleLedBuffer;
  AddressableLEDBuffer redLEDBuffer;
  AddressableLEDBuffer rainbowLEDBuffer;
  Timer ledTimerOff;
  Timer ledTimerOn;
  boolean blink = false; // sets robot to cone mode by default

  */
/** Creates a new LEDModeSubsystem. *//*

  public LEDModeSubsystem(Gripper gripper) {
    this.gripper = gripper;

    //create led blink timers
    ledTimerOff = new Timer();
    ledTimerOn = new Timer();

    // initialize LED's LED's
    led = new AddressableLED(PortConstants.kLEDStripPort);
    ledBufferOff = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    purpleLedBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    yellowLedBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    redLEDBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    rainbowLEDBuffer = new AddressableLEDBuffer(OIConstants.kLEDStripLength);
    led.setLength(yellowLedBuffer.getLength());
    led.start();

    // create buffers for each color
    for (var i = 0; i < purpleLedBuffer.getLength(); i++){
      purpleLedBuffer.setRGB(i, 255, 0, 255);
    }
    for (var i = 0; i < yellowLedBuffer.getLength(); i++){
      //yellowLedBuffer.setRGB(i, 255, 255, 0);
      yellowLedBuffer.setLED(i, Color.kYellow);
    }
    for (var i = 0; i < ledBufferOff.getLength(); i++){
      ledBufferOff.setRGB(i, 0, 0, 0);// off
    }
    for (var i = 0; i < redLEDBuffer.getLength(); i++){
      redLEDBuffer.setRGB(i, 255, 0, 0);// red
    }



    // defualt robot to Cone Mode
    led.setData(yellowLedBuffer);

    //led blinking timers
    ledTimerOn.start();
    ledTimerOff.reset();
    ledTimerOn.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Robot Mode", cubeMode);
    SmartDashboard.putNumber("Led On Timer", ledTimerOn.get());
    SmartDashboard.putNumber("Led Off Timer", ledTimerOff.get());


      //blink mode - Blink LED's for given time
      if (blink){
        if (ledTimerOn.get() >= .25){
          ledTimerOn.reset();
          ledTimerOn.stop();
          ledTimerOff.start();
          setLEDColor();
        }

        else if (ledTimerOff.get() >= .25){
          ledTimerOff.reset();
          ledTimerOff.stop();
          ledTimerOn.start();
          ledOff();
        }
      }

      // No game piece set led color based on mode
      else {
        setLEDColor();
      }

  }


  //set robot to cubme mode
  public void cubeMode() {
    cubeMode = true;

    led.setData(purpleLedBuffer);
  }

  // set robot to cone mode
  public void coneMode() {
    cubeMode = false;

    led.setData(yellowLedBuffer);
  }

  // Toggles the robot mode
  public void toggleRobotMode() {
    if (cubeMode){
      cubeMode = false;
    }
    else {
      cubeMode = true;
    }
  }

  // Get robot Mode true = cube false = cone
  public boolean getRobotMode() {
    return cubeMode;
  }

  // Set LED Color based on mode
  public void setLEDColor() {

    if (DriverStation.isDisabled()){
      led.setData(redLEDBuffer);
    }

    else if(cubeMode) {
      led.setData(purpleLedBuffer);
    }

    else{
      led.setData(yellowLedBuffer);
    }


  }

  // turn LED's off
  public void ledOff(){
    led.setData(ledBufferOff);
  }

  public void setRedLEDs () {
    led.setData(redLEDBuffer);
  }

  public void startBlinking() {// To be called elsewhere to blink the lights
    blink = true;
  }

  public void stopBlinking() {// To be called elsewhere to return the lights to a solid color
    blink = false;
  }

}
*/
////
//  // Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot;
//
//import static edu.wpi.first.units.Units.Meters;
//import static edu.wpi.first.units.Units.MetersPerSecond;
//
//import edu.wpi.first.units.measure.Distance;
//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.AddressableLEDBufferView;
//import edu.wpi.first.wpilibj.LEDPattern;
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
//
//
///**
// * The methods in this class are called automatically corresponding to each mode, as described in
// * the TimedRobot documentation. If you change the name of this class or the package after creating
// * this project, you must also update the Main.java file in the project.
// */
//public class Robot extends TimedRobot {
//  private static final String kDefaultAuto = "Default";
//  private static final String kCustomAuto = "My Auto";
//  private String m_autoSelected;
//  private final SendableChooser<String> m_chooser = new SendableChooser<>();
//
//  private AddressableLED m_led;
//  private AddressableLEDBuffer m_ledBuffer;
//
//  private double currentTime;
//
//  // Create the buffer
//AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(300);
//
//// Create the view for the section of the strip on the left side of the robot.
//// This section smmmpans LEDs from index 0 through index 59, inclusive.
//AddressableLEDBufferView m_left = m_buffer.createView(0, 150);
//
//// The section of the strip on the right side of the robot.
//// This section spans LEDs from index 60 through index 119, inclusive.
//// This view is reversed to cancel out the serpentine arrangement of the
//// physical LED strip on the robot.
//AddressableLEDBufferView m_right = m_buffer.createView(151, 299).reversed();
//
//// Create an LED pattern that sets the entire strip to solid
//LEDPattern red = LEDPattern.solid(Color.kRed);
//LEDPattern green = LEDPattern.solid(Color.kGreen);
//LEDPattern blue = LEDPattern.solid(Color.kBlue);
//LEDPattern white = LEDPattern.solid(Color.kWhite);
//
//  private Timer timer = new Timer();
//
//
////RANBOW DECLERATIONS
//  // all hues at maximum saturation and half brightness
//  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
//
//  // Our LED strip has a density of 120 LEDs per meter
//  private static final Distance kLedSpacing = Meters.of(1 / 30.0);
//
//  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
//  // of 1 meter per second.
//  private final LEDPattern m_scrollingRainbow =
//      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
//
//  /**
//  * This function is run when the robot is first started up and should be used for any
//   * initialization code.
//   */
//  public Robot() {
//    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
//    m_chooser.addOption("My Auto", kCustomAuto);
//    SmartDashboard.putData("Auto choices", m_chooser);
//  }
//
//  /**
//   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
//   * that you want ran during disabled, autonomous, teleoperated and test.
//   *
//   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
//   * SmartDashboard integrated updating.
//   */
//  @Override
//  public void robotPeriodic() {
//        // // Update the buffer with the rainbow animation
//        // m_scrollingRainbow.applyTo(m_ledBuffer);
//        // // Set the LEDs
//        // m_led.setData(m_ledBuffer);
//  }
//
//  /**
//   * This autonomous (along with the chooser code above) shows how to select between different
//   * autonomous modes using the dashboard. The sendable chooser code works with the Java
//   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
//   * uncomment the getString line to get the auto name from the text box below the Gyro
//   *
//   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
//   * below with additional strings. If using the SendableChooser make sure to add them to the
//   * chooser code above as well.
//   */
//  @Override
//  public void autonomousInit() {
//    m_autoSelected = m_chooser.getSelected();
//    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
//    System.out.println("Auto selected: " + m_autoSelected);
//
//
//    // Must be a PWM header, not MXP or DIO
//    m_led = new AddressableLED(0); //0 -> 9, top -> bottom
//
//    // Reuse buffer
//    // Default to a length of 60, start empty output
//    // Length is expensive to set, so only set it once, then just update data
//    m_ledBuffer = new AddressableLEDBuffer(300);
//    m_led.setLength(m_ledBuffer.getLength());
//
//    // Set the data
//    m_led.setData(m_ledBuffer);
//    m_led.start();
//
//    timer.reset();
//    timer.start();
//
//
//    green.applyTo(m_ledBuffer);
//    m_autoSelected = m_chooser.getSelected();
//    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
//    System.out.println("Auto selected: " + m_autoSelected);
//
//
//    // Must be a PWM header, not MXP or DIO
//    m_led = new AddressableLED(0); //0 -> 9, top -> bottom
//
//    // Reuse buffer
//    // Default to a length of 60, start empty output
//    // Length is expensive to set, so only set it once, then just update data
//    m_ledBuffer = new AddressableLEDBuffer(300);
//    m_led.setLength(m_ledBuffer.getLength());
//
//    // Set the data
//    m_led.setData(m_ledBuffer);
//    m_led.start();
//
//    timer.reset();
//    timer.start();
//
//
//    green.applyTo(m_ledBuffer);
//
//// Write the data to the LED strip
//m_led.setData(m_ledBuffer);
//
//  }
//
//
//  /** This function is called periodically during autonomous. */
//  @Override
//  public void autonomousPeriodic() {
//    switch (m_autoSelected) {
//      case kCustomAuto:
//        // Put custom auto code here
//        break;
//      case kDefaultAuto:
//      default:
//        // Put default auto code here
//        break;
//    }
//            // Update the buffer with the rainbow animation
//            // m_scrollingRainbow.applyTo(m_ledBuffer);                SET TO RANBOW
//            // Set the LEDs
//            m_led.setData(m_ledBuffer);
//
//            if (timer.get() < 3) {
//             green.applyTo(m_ledBuffer);
//              m_led.setData(m_ledBuffer);
//            }
//
//            else {
//              white.applyTo(m_ledBuffer);
//              m_led.setData(m_ledBuffer);
//            }
//
//  }
//
//  /** This function is called once when teleop is enabled. */
//  @Override
//  public void teleopInit() {}
//
//  /** This function is called periodically during operator control. */
//  @Override
//  public void teleopPeriodic() {}
//
//  /** This function is called once when the robot is disabled. */
//  @Override
//  public void disabledInit() {}
//
//  /** This function is called periodically when disabled. */
//  @Override
//  public void disabledPeriodic() {}
//
//  /** This function is called once when test mode is enabled. */
//  @Override
//  public void testInit() {}
//
//  /** This function is called periodically during test mode. */
//  @Override
//  public void testPeriodic() {}
//
//  /** This function is called once when the robot is first started up. */
//  @Override
//  public void simulationInit() {}
//
//  /** This function is called periodically whilst in simulation. */
//  @Override
//  public void simulationPeriodic() {}
//}

}