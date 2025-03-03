package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(CoralConstants.MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(CoralConstants.LIMIT_SWITCH_ID);

    private boolean unjamming = false;
    private boolean ejecting = false;

    public void unjam(boolean state) {
        unjamming = state;
    }

    public void eject(boolean state) {
        ejecting = state;
    }

    @Override
    public void periodic() {
        if (unjamming) {
            motor.set(CoralConstants.UNJAM_SPEED);
        } else if (ejecting) {
            motor.set(CoralConstants.EJECT_SPEED);
        } else if (!limitSwitch.get()) {
            motor.set(CoralConstants.IDLE_SPEED);
        } else {
            motor.stopMotor();
        }
    }
}
