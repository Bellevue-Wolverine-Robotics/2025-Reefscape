package frc.robot.constants;

public class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final ControlMode CONTROL_MODE = ControlMode.FULL_OPERATOR;

    public enum ControlMode {
        FULL_OPERATOR,
        PARTIAL_OPERATOR
    }
}
