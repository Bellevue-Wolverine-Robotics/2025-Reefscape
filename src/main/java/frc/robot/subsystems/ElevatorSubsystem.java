package frc.robot.subsystems;

import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
    private final DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_PORT);
    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TOP_LIMIT_SWITCH_PORT);
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final PIDController pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    private final Encoder encoder = new Encoder(1, 2, false, EncodingType.k4X);

    private double targetPosition = 0.0d;

    public ElevatorSubsystem() {
        encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);
    }

    public void setPosition(double newPosition) {
        targetPosition = newPosition;
    }

    @Override
    public void periodic() {
        var speed = pid.calculate(encoder.getDistance(), targetPosition);

        if (bottomLimitSwitch.get())
        {
            encoder.reset();
            if (speed < 0)
            {
                StopMotors();
                return;
            }
        }

        else if (topLimitSwitch.get() && speed > 0)
        {
            StopMotors();
            return;
        }

        SetMotors(speed);
    }

    private void StopMotors()
    {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    private void SetMotors(double speed)
    {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
