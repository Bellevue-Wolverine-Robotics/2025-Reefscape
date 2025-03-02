package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED subsystem for robot status indication.
 * Controls LED colors/patterns for states like game piece detection, AprilTag tracking, and robot status (disabled, enabled).
 */
public class LEDModeSubsystem extends SubsystemBase {
    // LED Hardware
    private AddressableLED ledStrip; // LED strip
    private AddressableLEDBuffer ledColors; // LED colors buffer

    // Color Buffers
    private AddressableLEDBuffer yellowBuffer; // 'Coral' (game piece) - Yellow
    private AddressableLEDBuffer blueBuffer;   // No 'Coral' - Blue
    private AddressableLEDBuffer redBuffer;    // Robot disabled - Red
    private AddressableLEDBuffer greenBuffer;  // AprilTag tracked - Solid Green
    private AddressableLEDBuffer flashingGreenBuffer; // AprilTag not tracked - Flashing Green

    // Blinking Timers
    private Timer blinkTimerOn;  // Blinking 'on' timer
    private Timer blinkTimerOff; // Blinking 'off' timer

    // Robot State
    private boolean hasCoral = false;       // 'Coral' game piece detected
    private boolean isTrackingAprilTag = false; // AprilTag tracking status
    private boolean shouldBlink = false;   // LED blink control

    /**
     * LEDModeSubsystem constructor.
     * Initializes LED strip, color buffers, and timers.
     */
    public LEDModeSubsystem() {
        // Initialize LED Strip
        ledStrip = new AddressableLED(1); // PWM port 1 (CHANGE IF DIFFERENT)
        ledColors = new AddressableLEDBuffer(150); // 150 LEDs (CHANGE IF DIFFERENT)
        ledStrip.setLength(ledColors.getLength());
        ledStrip.start();

        // Initialize Color Buffers
        yellowBuffer = new AddressableLEDBuffer(ledColors.getLength());
        blueBuffer = new AddressableLEDBuffer(ledColors.getLength());
        redBuffer = new AddressableLEDBuffer(ledColors.getLength());
        greenBuffer = new AddressableLEDBuffer(ledColors.getLength());
        flashingGreenBuffer = new AddressableLEDBuffer(ledColors.getLength());

        // Set Buffer Colors
        for (int i = 0; i < yellowBuffer.getLength(); i++) {
            yellowBuffer.setLED(i, Color.kYellow); // Yellow
        }
        for (int i = 0; i < blueBuffer.getLength(); i++) {
            blueBuffer.setRGB(i, 0, 0, 255); // Blue
        }
        for (int i = 0; i < redBuffer.getLength(); i++) {
            redBuffer.setRGB(i, 255, 0, 0); // Red
        }
        for (int i = 0; i < greenBuffer.getLength(); i++) {
            greenBuffer.setRGB(i, 0, 255, 0); // Green
        }
        for (int i = 0; i < flashingGreenBuffer.getLength(); i++) {
            if (i % 2 == 0) {
                flashingGreenBuffer.setRGB(i, 0, 255, 0); // Green (even LEDs)
            } else {
                flashingGreenBuffer.setRGB(i, 0, 0, 0);   // Off (odd LEDs)
            }
        }

        // Initialize Blinking Timers
        blinkTimerOn = new Timer();
        blinkTimerOff = new Timer();

        // Initial LED Color
        updateLEDs(); // Set initial LED color
    }

    @Override
    public void periodic() {
        // Runs every robot cycle (e.g., 20ms)

        // SmartDashboard Debugging
//        SmartDashboard.putBoolean("Coral Sensor Active", hasCoral());
//        SmartDashboard.putBoolean("AprilTag Is Tracked", isAprilTagTracked());
//        SmartDashboard.putNumber("Blink Timer On", blinkTimerOn.get());
//        /*SmartDashboard.putNumber("Blink Timer Off", blinkTim/**/erOff.get());*/
        
        //   Handle LED Blinking
        if   (shouldBlink) {
                if (blinkTimerOn.isRunning() && blinkTimerOn.get() >= 0.25) {
                blinkTimerOn.stop();
                blinkTimerOff.reset();
                blinkTimerOff.start();
                updateLEDs(); // Flashing green ON
            } else if (blinkTimerOff.isRunning() && blinkTimerOff.get() >= 0.25) {
                blinkTimerOff.stop();
                blinkTimerOn.reset();
                blinkTimerOn.start();
                turnLEDsOff(); // LEDs off (blinking)
            }
        } else {
            updateLEDs(); // Update LEDs (no blinking)
        }
    }

    // LED Control Methods

    /**
     * Sets LED color based on robot status.
     * Chooses color buffer based on disabled state, 'Coral' detection, or AprilTag tracking.
     */
    public void updateLEDs() {
        if (DriverStation.isDisabled()) {
            ledStrip.setData(flashingGreenBuffer); // Flash green when disabled
            shouldBlink = true; // Enable blinking

            if (!blinkTimerOn.isRunning() && !blinkTimerOff.isRunning()) {
                blinkTimerOn.reset();
                blinkTimerOff.reset();
                blinkTimerOn.start(); // Start blinking timer
            }

        } else if (hasCoral()) {
            ledStrip.setData(yellowBuffer); // Yellow when 'Coral' detected
            shouldBlink = false; // No blinking
        } else if (isAprilTagTracked()) {
            ledStrip.setData(greenBuffer); // Solid green when AprilTag tracked
            shouldBlink = false; // No blinking
        } else {
            ledStrip.setData(flashingGreenBuffer); // Flash green if AprilTag not tracked
            shouldBlink = true; // Enable blinking

            if (!blinkTimerOn.isRunning() && !blinkTimerOff.isRunning()) {
                blinkTimerOn.reset();
                blinkTimerOff.reset();
                blinkTimerOn.start(); // Start blinking timer
            }
        }

        if (!shouldBlink) {
            blinkTimerOn.stop();
            blinkTimerOff.stop();
            blinkTimerOn.reset();
            blinkTimerOff.reset();
        }
    }


    /**
     * Turns off LEDs (red for testing).
     * Sets LEDs to red for testing; consider black (offBuffer) for final.
     */
    public void turnLEDsOff() {
        ledStrip.setData(redBuffer); // Red for testing, ledColors is black
    }


    // Setter Methods

    /**
     * Sets 'Coral' sensor status.
     * @param coralDetected True if detected, false otherwise.
     */
    public void setHasCoral(boolean coralDetected) {
        hasCoral = coralDetected;
        updateLEDs();
    }

    /**
     * Sets AprilTag tracking status.
     * @param aprilTagIsTracked True if tracked, false otherwise.
     */
    public void setIsAprilTagTracked(boolean aprilTagIsTracked) {
        isTrackingAprilTag = aprilTagIsTracked;
        updateLEDs();
    }


    // Getter Methods

    /**
     * Checks if 'Coral' is detected.
     * @return True if 'Coral' detected, false otherwise.
     */
    public boolean hasCoral() {
        return hasCoral; // Placeholder - Replace with sensor logic
    }

    /**
     * Checks if AprilTag is tracked.
     * @return True if AprilTag tracked, false otherwise.
     */
    public boolean isAprilTagTracked() {
        return isTrackingAprilTag; // Placeholder - Replace with vision system logic
    }
}