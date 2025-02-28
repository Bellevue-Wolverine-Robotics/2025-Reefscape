package frc.robot.subsystems;

import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
    private static final double RolloverThreshold = 0.75d;

    private final DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchPort);
    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchPort);
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.leftMotorId, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.rightMotorId, MotorType.kBrushless);
    private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(0);

    private double position = 0.0d;
    private double movementDir;
    private double rotation;

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
        var previousRotation = rotation;
        var currentRotation = dutyCycleEncoder.get();
        var change = currentRotation - previousRotation;

        if (Math.abs(change) >= RolloverThreshold) {
            change = (1 - (Math.abs(change))) * -Math.signum(change);
        }

        movementDir = Math.signum(change);

        position += change;
        rotation = currentRotation;
    }
}
