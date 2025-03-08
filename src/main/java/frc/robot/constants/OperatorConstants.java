package frc.robot.constants;

public class OperatorConstants {

    // Joystick Deadband
    public static final double kDeadzone = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // Xbox Controller Axis
    public static final int kControllerLeftX = 0;
    public static final int kControllerLeftY = 1;
    public static final int kControllerRightX = 2;
    public static final int kControllerRightY = 3;

    // Xbox Controller Buttons
    public static final int kControllerRightTrigger = 8;
    public static final int kControllerLeftTrigger = 9;

    // Controllers
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final ControlMode CONTROL_MODE = ControlMode.FULL_OPERATOR;

    public enum ControlMode {
        FULL_OPERATOR,
        PARTIAL_OPERATOR
    }
}
