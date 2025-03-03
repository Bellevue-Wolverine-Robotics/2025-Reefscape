package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_PORT);
    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TOP_LIMIT_SWITCH_PORT);
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final PIDController pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    private final Encoder encoder = new Encoder(1, 2, false, EncodingType.k4X);

    private double nextPosition = 0.0d;
    private double targetPosition = 0.0d;

    public ElevatorSubsystem() {
        var config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid.setTolerance(ElevatorConstants.ERROR_TOLERANCE, ElevatorConstants.ERROR_DERIVATIVE_TOLERANCE);
        encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);
    }

    public void setTarget(double position) {
        nextPosition = position;
    }

    public void adjust() {
        targetPosition = nextPosition;
    }

    @Override
    public void periodic() {
        var speed = pid.calculate(encoder.getDistance(), targetPosition);

        if (bottomLimitSwitch.get()) {
            encoder.reset();
        }

        if (bottomLimitSwitch.get() && speed < 0 || topLimitSwitch.get() & speed > 0) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            leftMotor.set(speed);
            rightMotor.set(speed);
        }
    }
}
