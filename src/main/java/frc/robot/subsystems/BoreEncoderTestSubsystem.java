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

    private double movementDir;
    private double speed = 0.1d;
    private double position = 0.0d;
    private double targetHeight = 0.0d;

    Encoder quadratureEncoder;
    DutyCycleEncoder dutyCycleEncoder;
    int i = 0;

    public BoreEncoderTestSubsystem() {
        leftPulleyMotor = new SparkMax(4, MotorType.kBrushless);
        rightPulleyMotor = new SparkMax(2, MotorType.kBrushless);

        leftPulleyMotor.setInverted(true);
        rightPulleyMotor.setInverted(true);
        
        quadratureEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
        dutyCycleEncoder = new DutyCycleEncoder(0);
        quadratureEncoder.setDistancePerPulse(1.0d);
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
        i++;
        if (i == 3)
        {
            //System.out.println("from periodic: " + dutyCycleEncoder.get());
            //System.out.println("enabled: " + dutyCycleEncoder.isConnected());
            System.out.println("from periodic: " + quadratureEncoder.getDistance());
            i = 0;
        }
    }

    private void setBothMotors(double speed)
    {
        leftPulleyMotor.set(speed);
        rightPulleyMotor.set(speed);
    }
}
