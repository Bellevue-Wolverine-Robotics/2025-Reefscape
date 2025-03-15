package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(CoralConstants.MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(CoralConstants.LIMIT_SWITCH_ID);

    public CoralSubsystem() {
        // Invert the motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.setDefaultCommand(intake());
    }

    private Command intake() {
        return Commands.run(
            () -> {
                if (limitSwitch.get()) {
                    motor.stopMotor();
                } else {
                    motor.set(CoralConstants.IDLE_SPEED);
                }
            },
            this
        );
    }

    public Command eject() {
        return Commands.run(() -> motor.set(CoralConstants.EJECT_SPEED), this);
    }

    public Command unjam() {
        return Commands.run(() -> motor.set(CoralConstants.UNJAM_SPEED), this);
    }

    @Override
    public void periodic() {
        var loaded = limitSwitch.get();
        LEDModeSubsystem.setHasCoral(loaded);
    }
}
