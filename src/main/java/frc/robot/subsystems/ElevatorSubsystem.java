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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leaderMotor = new SparkMax(ElevatorConstants.UP_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax followerMotor = new SparkMax(ElevatorConstants.DOWN_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_PORT);
    private final PIDController pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    private final Encoder encoder = new Encoder(ElevatorConstants.ENCODER_CHANNEL_A, ElevatorConstants.ENCODER_CHANNEL_B, false, EncodingType.k4X);

    private double scoringPosition = 0.0d;

    public ElevatorSubsystem() {
        var leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        followerConfig.follow(leaderMotor);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid.setTolerance(ElevatorConstants.ERROR_TOLERANCE);
        encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);
        this.setDefaultCommand(holdIntakeLevel());
    }

    private Command holdIntakeLevel() {
        return Commands.run(() -> holdPosition(ElevatorConstants.INTAKE_LEVEL), this);
    }

    public Command holdScoringLevel() {
        return Commands.run(() -> holdPosition(scoringPosition), this);
    }

    public Command setScoringPosition(double position) {
        return Commands.runOnce(() -> scoringPosition = position);
    }

    private void holdPosition(double position) {
        var speed = pid.calculate(encoder.getDistance(), position);

        if (limitSwitch.get()) {
            encoder.reset();
        }

        if (limitSwitch.get() && speed < 0) {
            leaderMotor.stopMotor();
        } else {
            leaderMotor.set(speed);
        }
    }
}
