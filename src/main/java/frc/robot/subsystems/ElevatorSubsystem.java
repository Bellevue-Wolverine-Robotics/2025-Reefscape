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
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);

    private double scoringPosition = 0.0d;
    private boolean overrided = false;

    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
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
                double axis = -controllerAxisSupplier.getAsDouble();
                System.out.println(encoder.getDistance());

                if (axis > 0) {
                    motor.set(axis);
                } else {
                    motor.set(axis);
                }
            },
            this
        );
    }

    private static final double SPEED_DEADBAND = 0.1;
    private static final double CREEP_SPEED = 1; //creep speed

    private void movePosition(double position) {
        double distance = encoder.getDistance();
        double speed = pid.calculate(distance, position);

        boolean atBottom = limitSwitch.get();

        // If PID output is tiny, creep toward bottom only if we still need to descend and haven't hit switch
        if (Math.abs(speed) < SPEED_DEADBAND) {
            if (!atBottom && position < distance) {
                speed = -CREEP_SPEED;
            } else {
                speed = 0.0;
            }
        }

        // Do not drive further down once bottom switch is hit
        if (atBottom && speed < 0) {
            speed = 0.0;
        }

        motor.set(speed);
    }
}
