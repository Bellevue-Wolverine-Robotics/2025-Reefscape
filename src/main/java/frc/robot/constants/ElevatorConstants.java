package frc.robot.constants;

public final class ElevatorConstants {
    public static final double DISTANCE_PER_PULSE = Math.PI * 0.75d / 2048.0d; // // 0.75" is the diameter, circumference = 2d, 2048 is the bore encoder resolution (cycles per rev)

    public static final int MOTOR_ID = 31;
    public static final int ENCODER_CHANNEL_A = 1;
    public static final int ENCODER_CHANNEL_B = 2;

    public static final double KP = .35d;
    public static final double KI = 0.0d;
    public static final double KD = 0.0d;
    public static final double ERROR_TOLERANCE = 0.5d;

    public static final double LEVEL_ZERO = 0.0d;
    public static final double LEVEL_ONE = 15.0d;
    public static final double LEVEL_TWO = 20.0d;
    public static final double LEVEL_THREE = 34.0d;
    public static final double LEVEL_FOUR = 52.0d;

    public static final double UP_AXIS_COEFFICIENT = 0.8d;
    public static final double DOWN_AXIS_COEFFICIENT = 0.8d;
}
