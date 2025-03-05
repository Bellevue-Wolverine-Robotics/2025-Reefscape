package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.LEDConstants;

/**
 * LED subsystem for robot status indication.
 * Controls LED colors/patterns for states like game piece detection, AprilTag tracking, and robot status (disabled, enabled).
 */
public class LEDModeSubsystem extends SubsystemBase {
    private static boolean hasCoral = false;
    private static boolean isTrackingAprilTag = false;
    private final AddressableLED led = new AddressableLED(LEDConstants.PWM_PORT); // LED strip
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.LENGTH); // LED colors buffer

    /**
     * LEDModeSubsystem constructor.
     * Initializes LED strip, color buffers, and timers.
     */
    public LEDModeSubsystem() {
        led.setLength(ledBuffer.getLength());
        led.start();
    }

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

    @Override
    public void periodic() {
        updateLEDs();
    }

    /**
     * Sets LEDPattern color & blinking based on robot status.
     * Blue if we have coral, yellow if not, blinking if tracking an april tag, stable if not.
     * Finally, green if the robot is disabled.
     */
    private void updateLEDs() {
        LEDPattern pattern;

        if (RobotState.isDisabled()) {
            pattern = LEDConstants.Patterns.DISABLED;
        } else {
            if (hasCoral) pattern = LEDConstants.Patterns.HAS_CORAL;
            else pattern = LEDConstants.Patterns.NO_CORAL;
            if (isTrackingAprilTag) pattern = pattern.blink(LEDConstants.BLINK_SPEED);
        }

        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
