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

    private double speed = 0.03d;

    private double position = 0.0d;
    private double previousRotation;

    private double targetPosition = 0;
    // false is down, true is up
    private boolean targetDirection = false;

    /**
     * Sets the target position of the elevator, aka where it is trying to go to.
     * @param targetPosition The position the elevator will start moving to.
     * @exception 
     */
    public void setTargetPosition(int targetPosition) {
        // check targetPosition is in range
        if (targetPosition < ElevatorConstants.BOTTOM_POSITION || targetPosition > ElevatorConstants.TOP_POSITION) return;
        this.targetPosition = targetPosition;
        targetDirection = targetPosition - position > 0;
    }

    @Override
    public void periodic() {
        UpdatePosition();

        if (bottomLimitSwitch.get()) {
            position = ElevatorConstants.BOTTOM_POSITION;
        }
        else if (topLimitSwitch.get()) {
            position = ElevatorConstants.TOP_POSITION;
        }

        
        var movementDirection = targetPosition - position > 0;
        if (movementDirection != targetDirection) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            double velocity = movementDirection ? speed : -speed;
            leftMotor.set(velocity);
            rightMotor.set(velocity);
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
