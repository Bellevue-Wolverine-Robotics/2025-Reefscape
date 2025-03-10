package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private int direction = 0;
    private boolean overrided = false;
    //int i = 0;
    private final SparkMax motor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);
    private final PIDController pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    private final Encoder encoder = new Encoder(ElevatorConstants.ENCODER_CHANNEL_A, ElevatorConstants.ENCODER_CHANNEL_B, false, EncodingType.k4X);

    private double scoringPosition = 0.0d;
    public ElevatorSubsystem(CommandXboxController operator) {
        var config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid.setTolerance(ElevatorConstants.ERROR_TOLERANCE);
        encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);

        //operator.leftBumper().whileTrue(openLoop(ElevatorConstants.UP_SPEED)).onFalse(openLoop(0));
        //operator.rightBumper().whileTrue(openLoop(-ElevatorConstants.DOWN_SPEED)).onFalse(openLoop(0));
    }

    private Command maintainPosition() {
        return Commands.run(
            () -> {
                switch (OperatorConstants.CONTROL_MODE) {
                    case FULL_OPERATOR:
                        goToPosition(scoringPosition);
                        break;
                    case PARTIAL_OPERATOR:
                        goToPosition(ElevatorConstants.INTAKE_LEVEL);
                        break;
                }
            },
            this
        );
    }

    public Command openLoop(double input) {
        System.out.println(encoder.getDistance());
        if (input < 0.0d && encoder.getDistance() >= 0.0d) return runOnce(() -> motor.set(0));
        return runOnce(() -> motor.set(input));
    }

    public Command holdScoringPosition() {
        return Commands.run(() -> goToPosition(scoringPosition), this);
    }

    public Command setScoringPosition(double position) {
        direction = position > encoder.get() ? 1 : -1;
        return Commands.runOnce(() -> scoringPosition = position);
    }

    public Command decreaseScoringPosition() {
        return Commands.run(() -> scoringPosition -= ElevatorConstants.INCREMENT_DISTANCE);
    }

    public Command increaseScoringPosition() {
        return Commands.run(() -> scoringPosition += ElevatorConstants.INCREMENT_DISTANCE);
    }

    /* 
    @Override
    public void periodic()
    {
        // if override
        if (overrided) return;
        if (scoringPosition < encoder.getDistance() && direction == 1)
        {
            motor.set(ElevatorConstants.DOWN_SPEED);
        }
    }
    */

    private void goToPosition(double position) {
        var speed = pid.calculate(encoder.getDistance(), position);

        if (limitSwitch.get()) {
            encoder.reset();
        }

        if (limitSwitch.get() && speed < 0) {
            motor.stopMotor();
        } else {
            motor.set(speed);
        }
    }


}
