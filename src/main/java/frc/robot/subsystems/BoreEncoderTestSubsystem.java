package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BoreEncoderTestSubsystem extends SubsystemBase {
    private final SparkMax leftPulleyMotor;
    private final SparkMax rightPulleyMotor;

    // These nums are temporary and meaningless
    public static final double L1 = 25.0d;
    public static final double L2 = 55.0d;
    public static final double L3 = 75.0d;
    public static final double L4 = 101.0d;

    public static final double RolloverThreshold = 0.75d;

    private double movementDir;
    private double rotation;
    private double speed = 0.1d;
    private double position = 0.0d;
    private double targetHeight = 0.0d;

    private final DutyCycleEncoder dutyCycleEncoder;
    private int i = 0;

    public BoreEncoderTestSubsystem() {
        leftPulleyMotor = new SparkMax(4, MotorType.kBrushless);
        rightPulleyMotor = new SparkMax(2, MotorType.kBrushless);

        leftPulleyMotor.setInverted(true);
        rightPulleyMotor.setInverted(true);
        
        dutyCycleEncoder = new DutyCycleEncoder(0);
    }

    public double getCurrentRotation() {
        return dutyCycleEncoder.get();
    }

    public void goToHeight(double height) {
        movementDir = Math.signum(height - position);
        setBothMotors(speed * movementDir);
        targetHeight = height;
    }

    @Override
    public void periodic() {
        IncreaseDist();
        rotation = getCurrentRotation();
        System.out.println(position);
    }

    private void setBothMotors(double speed)
    {
        leftPulleyMotor.set(speed);
        rightPulleyMotor.set(speed);
    }

    private void IncreaseDist()
    {
        var previousRotation = rotation;
        var currentRotation = getCurrentRotation();
        var change = currentRotation - previousRotation;
        var sign = Math.signum(change);

        if (Math.abs(change) >= RolloverThreshold)
        {
            change = (1 - (Math.abs(change))) * -sign;
            //System.out.println("Previous: " + rotation + ", Current: " + currentRotation + ", Change: " + change);
        }

        position += change;
    }
}
