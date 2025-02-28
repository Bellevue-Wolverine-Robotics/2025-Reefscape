package frc.robot.subsystems;

import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
    private final DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchPort);
    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchPort);
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.leftMotorId, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.rightMotorId, MotorType.kBrushless);

    private int targetPosition = 0;
    private boolean targetDirection = false;

    public int getPosition() {
        return 0;
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
        targetDirection = position - getPosition() > 0;
    }

    @Override
    public void periodic() {
        if (targetPosition - getPosition() > 0 != targetDirection || !bottomLimitSwitch.get() || !topLimitSwitch.get()) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        } else {
            double speed = targetPosition > 0 ? 0.01 : -0.01;
            leftMotor.set(speed);
            rightMotor.set(speed);
        }
    }
}
