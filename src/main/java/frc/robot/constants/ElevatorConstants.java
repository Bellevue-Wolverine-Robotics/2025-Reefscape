package frc.robot.constants;

public final class ElevatorConstants {
    public static final double CIRCUMFERENCE = Math.PI  * 0.75d; // 0.75" is the diameter, circumference = 2d
    public static final double DISTANCE_PER_PULSE = CIRCUMFERENCE / 2048.0d; // 2048 is the bore encoder resolution (cycles per rev)

    public static final int BOTTOM_LIMIT_SWITCH_PORT = 1;
    public static final int TOP_LIMIT_SWITCH_PORT = 2;

    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 1;

    public static final double KP = .03d;
    public static final double KI = 0.0d;
    public static final double KD = 0.0d;
    public static final int ERROR_TOLERANCE = 3;
    public static final int ERROR_DERIVATIVE_TOLERANCE = 10;

    public static final double LEVEL_ONE = 0.0d;
    public static final double LEVEL_TWO = 0.0d;
    public static final double LEVEL_THREE = 0.0d;
    public static final double LEVEL_FOUR = 0.0d;
    public static final double INTAKE_LEVEL = 0.0d;
}
