package frc.robot.subsystems;

import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
    private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(ElevatorConstants.DUTY_CYCLE_ENCODER_PORT);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_PORT);
    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TOP_LIMIT_SWITCH_PORT);
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    private double position = 0.0d;
    private double previousRotation;

    private double targetPosition = 0;
    private boolean targetDirection = false;

    public void setTargetPosition(int position) {
        targetPosition = position;
        targetDirection = position - position > 0;
    }

    @Override
    public void periodic() {
        UpdatePosition();

        if (targetPosition - position > 0 != targetDirection || !bottomLimitSwitch.get() || !topLimitSwitch.get()) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            double speed = targetPosition > 0 ? 0.01 : -0.01;
            leftMotor.set(speed);
            rightMotor.set(speed);
        }
    }

    private void UpdatePosition() {
        var currentRotation = dutyCycleEncoder.get();
        var change = currentRotation - previousRotation;

        if (Math.abs(change) >= ElevatorConstants.ROLLOVER_THRESHOLD) {
            change = (1 - (Math.abs(change))) * -Math.signum(change);
        }

        previousRotation = currentRotation;
    }
}
