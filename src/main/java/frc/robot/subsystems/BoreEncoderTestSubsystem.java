package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BoreEncoderTestSubsystem extends SubsystemBase {
    private static final double RolloverThreshold = 0.75d;
    private double position = 0.0d;
    private double movementDir;
    private double rotation;
    private final DutyCycleEncoder dutyCycleEncoder;

    public BoreEncoderTestSubsystem() {
        dutyCycleEncoder = new DutyCycleEncoder(0);
    }

    public double getCurrentRotation() {
        return dutyCycleEncoder.get();
    }

    @Override
    public void periodic() {
        UpdatePosition();
    }

    private void UpdatePosition() {
        var previousRotation = rotation;
        var currentRotation = getCurrentRotation();
        var change = currentRotation - previousRotation;

        if (Math.abs(change) >= RolloverThreshold) {
            change = (1 - (Math.abs(change))) * -Math.signum(change);
        }

        movementDir = Math.signum(change);

        position += change;
        rotation = currentRotation;
    }
}
