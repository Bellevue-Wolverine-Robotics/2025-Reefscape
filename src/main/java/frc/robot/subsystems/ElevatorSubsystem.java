package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
    private final PIDController pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    private final Encoder encoder = new Encoder(ElevatorConstants.ENCODER_CHANNEL_A, ElevatorConstants.ENCODER_CHANNEL_B, false, EncodingType.k4X);

    private double scoringPosition = 0.0d;
    private boolean overrided = false;

    public ElevatorSubsystem() {
        var config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid.setTolerance(ElevatorConstants.ERROR_TOLERANCE);
        encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);

        this.setDefaultCommand(maintainPosition());
    }

    private Command maintainPosition() {
        return Commands.run(
            () -> {
                if (overrided) {
                    motor.stopMotor();
                } else {
                    switch (OperatorConstants.CONTROL_MODE) {
                        case FULL_OPERATOR:
                            movePosition(scoringPosition);
                            break;
                        case PARTIAL_OPERATOR:
                            movePosition(ElevatorConstants.LEVEL_ZERO);
                            break;
                    }
                }
            },
            this
        );
    }

    public Command holdScoringPosition() {
        return Commands.run(
            () -> movePosition(scoringPosition),
            this
        );
    }

    public Command setScoringPosition(double position) {
        return Commands.runOnce(
            () -> {
                overrided = false;
                scoringPosition = position;
            },
            this
        );
    }

    public Command moveManual(DoubleSupplier controllerAxisSupplier) {
        return Commands.run(
            () -> {
                overrided = true;
                var axis = controllerAxisSupplier.getAsDouble();
   
                if (axis > 0) {
                    motor.set((axis - OperatorConstants.kDeadzone) * ElevatorConstants.UP_AXIS_COEFFICIENT);
                } else {
                    motor.set((axis + OperatorConstants.kDeadzone) * ElevatorConstants.DOWN_AXIS_COEFFICIENT);
                }
            },
            this
        );
    }

    private void movePosition(double position) {
        var distance = encoder.getDistance();
        var speed = pid.calculate(distance, position);

        if (distance <= ElevatorConstants.LEVEL_ZERO && speed < 0 || distance >= ElevatorConstants.LEVEL_FOUR && speed > 0) {
            motor.stopMotor();
        } else {
            motor.set(speed);
        }
    }
}
