package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

/**
 * LED subsystem for robot status indication.
 * Controls LED colors/patterns for states like game piece detection, AprilTag tracking, and robot status (disabled, enabled).
 */
public class LEDModeSubsystem extends SubsystemBase {
    private static boolean hasCoral = false;       // 'Coral' game piece detected
    private static boolean isTrackingAprilTag = false; // AprilTag tracking status

    // LED Hardware
    private AddressableLED led; // LED strip
    private AddressableLEDBuffer ledBuffer; // LED colors buffer

    /**
     * LEDModeSubsystem constructor.
     * Initializes LED strip, color buffers, and timers.
     */
    public LEDModeSubsystem() {
        // Initialize LED Strip
        led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LENGTH);
        led.setLength(ledBuffer.getLength());
        led.start();

        // initial LED set
        updateLEDs();
    }

    @Override
    public void periodic() {
        updateLEDs();
    }

    /**
     * Turns off LEDs (red for testing).
     * Sets LEDs to red for testing; consider black (offBuffer) for final.
     */
    public void turnLEDsOff() {
        LEDPattern.solid(Color.kBlack).applyTo(ledBuffer); // Green for testing, ledBuffer is black
    }


    // Setter Methods

    /**
     * Sets 'Coral' sensor status.
     * @param coralDetected True if detected, false otherwise.
     */
    public static void setHasCoral(boolean coralDetected) {
        hasCoral = coralDetected;
    }

    /**
     * Sets AprilTag tracking status.
     * @param aprilTagIsTracked True if tracked, false otherwise.
     */
    public static void setAprilTagTracked(boolean aprilTagTracked) {
        isTrackingAprilTag = aprilTagTracked;
    }

    /**
     * Sets LEDPattern color & blinking based on robot status.
     * Blue if we have coral, yellow if not, blinking if tracking an april tag, stable if not.
     * Finally, green if the robot is disabled.
     */
    private void updateLEDs()
    {
        LEDPattern pattern;

        if (RobotState.isDisabled()) {
            pattern = LEDConstants.Patterns.DISABLED;
        }

        else
        {
            if (hasCoral) pattern = LEDConstants.Patterns.HAS_CORAL;
            else pattern = LEDConstants.Patterns.NO_CORAL;

            if (isTrackingAprilTag) pattern = pattern.blink(LEDConstants.BLINK_SPEED);
        }
        
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}