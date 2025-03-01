package frc.robot.constants;

public final class ElevatorConstants {
    // 0.75" is the diameter, circumference = 2d
    public static final double CIRCUMFERENCE = Math.PI  * 0.75d;

    // 2048 is the bore encoder resolution (cycles per rev)
    public static final double DISTANCE_PER_PULSE = CIRCUMFERENCE / 2048.0d;

    public static final double ROLLOVER_THRESHOLD = 0.75d;
    public static final double BOTTOM_POSITION = 0.0d;
    // change this later
    public static final double TOP_POSITION = 100.0d;

    public static final int DUTY_CYCLE_ENCODER_PORT = 0;
    public static final int BOTTOM_LIMIT_SWITCH_PORT = 2;
    public static final int TOP_LIMIT_SWITCH_PORT = 1;
    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 1;
    
}
