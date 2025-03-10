package frc.robot.constants;

public final class ElevatorConstants {
    public static final double CIRCUMFERENCE = Math.PI  * 0.75d; // 0.75" is the diameter, circumference = 2d
    public static final double DISTANCE_PER_PULSE = CIRCUMFERENCE / 2048.0d; // 2048 is the bore encoder resolution (cycles per rev)

    public static final int MOTOR_ID = 31;
    public static final int LIMIT_SWITCH_PORT = 3;
    public static final int ENCODER_CHANNEL_A = 1;
    public static final int ENCODER_CHANNEL_B = 2;

    public static final double KP = .03d;
    public static final double KI = 0.0d;
    public static final double KD = 0.0d;
    public static final int ERROR_TOLERANCE = 3;

    public static final double LEVEL_ONE = 2.0d;
    public static final double LEVEL_TWO = 3.0d;
    public static final double LEVEL_THREE = 6.0d;
    public static final double LEVEL_FOUR = 5.0d;
    public static final double INTAKE_LEVEL = 1.0d;
    public static final double BOTTOM_LEVEL = 0.0d;

    public static final double UP_SPEED = 0.6d;
    public static final double DOWN_SPEED = 0.4d;
}
