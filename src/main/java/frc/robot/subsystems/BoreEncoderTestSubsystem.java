package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BoreEncoderTestSubsystem extends SubsystemBase {
    DutyCycleEncoder boreEncoder;

    public BoreEncoderTestSubsystem() {
        boreEncoder = new DutyCycleEncoder(1);
    }

    @Override
    public void periodic() {
        System.out.println(boreEncoder.get());
    }
}
