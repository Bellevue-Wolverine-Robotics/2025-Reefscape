package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final Time BLINK_SPEED = Seconds.of(0.25d);
    public static final Distance LED_SPACING = Inches.of(1.0d);

    public static final int PWM_PORT = 9;
    public static final int LENGTH = 9 * 2;

    public final class Patterns {
        public static final LEDPattern HAS_CORAL = LEDPattern.solid(Color.kYellow);
        public static final LEDPattern NO_CORAL = LEDPattern.solid(Color.kBlue);
        public static final LEDPattern DISABLED = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0d), LED_SPACING);
    }
}
