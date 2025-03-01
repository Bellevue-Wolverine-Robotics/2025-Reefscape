package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class BoreEncoderTestSubsystem extends SubsystemBase {
    //private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(ElevatorConstants.DUTY_CYCLE_ENCODER_PORT);
    private final Encoder encoder;

    public BoreEncoderTestSubsystem()
    {
        encoder = new Encoder(1, 2, false, EncodingType.k4X);
        encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);
    }
    public void periodic() {
        System.out.println(encoder.getDistance());
    }
}
