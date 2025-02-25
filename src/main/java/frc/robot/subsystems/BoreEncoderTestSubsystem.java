package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BoreEncoderTestSubsystem extends SubsystemBase {
    Encoder quadratureEncoder;
    DutyCycleEncoder dutyCycleEncoder;
    int i = 0;

    public BoreEncoderTestSubsystem() {
        quadratureEncoder = new Encoder(1, 2);
        dutyCycleEncoder = new DutyCycleEncoder(0);
    }

    public double getCurrentRotation() {
        return dutyCycleEncoder.get();
    }

    @Override
    public void periodic() {
        i++;
        if (i == 30)
        {
            /*
            System.out.println("from periodic: " + dutyCycleEncoder.get());
            System.out.println("enabled: " + dutyCycleEncoder.isConnected());
            */
            System.out.println("from periodic: " + quadratureEncoder.get());
            i = 0;
        }
    }
}
