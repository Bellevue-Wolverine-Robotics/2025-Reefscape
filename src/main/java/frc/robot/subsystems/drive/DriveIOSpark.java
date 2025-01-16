package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.constants.DriveConstants;
import frc.robot.interfaces.DriveIO;

import static frc.robot.utils.SparkUtil.*;

import java.util.function.DoubleSupplier;

/** Sparkmax implementation for drivetrain */
public class DriveIOSpark implements DriveIO {
  // Motor Controllers
  private SparkMax leftLeader = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private SparkMax rightLeader = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private SparkMax leftFollower = new SparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
  private SparkMax rightFollower = new SparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

  // Encoders
  private RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private RelativeEncoder rightEncoder = rightLeader.getEncoder();

  // PID controllers
  private SparkClosedLoopController leftPIDController = leftLeader.getClosedLoopController();
  private SparkClosedLoopController rightPIDController = rightLeader.getClosedLoopController();

  /** Creates an instance of the sparkmax drivetrain controller system */
  public DriveIOSpark() {
    // Create Config
    var config = new SparkMaxConfig();

    // General config
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.kCurrentLimit)
        .voltageCompensation(DriveConstants.kVoltageCompensation);

    // PID config
    config.closedLoop
        .pidf(DriveConstants.realKp, 0.0, DriveConstants.realKd, 0.0);

    // Encoder config
    config.encoder
        .positionConversionFactor(DriveConstants.kWheelCircumferenceMeters / DriveConstants.kDriveGearRatio)
        .velocityConversionFactor((DriveConstants.kWheelCircumferenceMeters / DriveConstants.kDriveGearRatio) / 60);

    // Apply config to leaders
    config.inverted(DriveConstants.kLeftInverted);
    tryUntilOk(
        leftLeader,
        5,
        () -> leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    config.inverted(DriveConstants.kRightInverted);
    tryUntilOk(
        rightLeader,
        5,
        () -> rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Apply config to followers
    config.inverted(DriveConstants.kLeftInverted).follow(leftLeader);
    tryUntilOk(
        leftFollower,
        5,
        () -> leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    config.inverted(DriveConstants.kRightInverted).follow(rightLeader);
    tryUntilOk(
        rightFollower,
        5,
        () -> rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    /**
     * Applied Voltage Calculation:
     * Calculate and store the actual voltage being delivered to the motor.
     * This is done by multiplying the controller's output percentage (the fraction
     * of power being applied, ranging from -1 to 1)
     * by the input voltage supplied to the motor controller (the voltage coming
     * from the battery, typically around 12V).
     */

    ifOk(leftLeader, leftEncoder::getPosition, (value) -> inputs.leftPositionRad = value);
    ifOk(leftLeader, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRadPerSec = value);
    ifOk(leftLeader,
        new DoubleSupplier[] { leftLeader::getAppliedOutput, leftLeader::getBusVoltage },
        (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
    ifOk(
        leftLeader,
        new DoubleSupplier[] { leftLeader::getOutputCurrent, leftLeader::getOutputCurrent },
        (values) -> inputs.leftCurrentAmps = values);

    ifOk(rightLeader, rightEncoder::getPosition, (value) -> inputs.rightPositionRad = value);
    ifOk(rightLeader, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRadPerSec = value);
    ifOk(
        rightLeader,
        new DoubleSupplier[] { rightLeader::getAppliedOutput, rightLeader::getBusVoltage },
        (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
    ifOk(
        rightLeader,
        new DoubleSupplier[] { rightLeader::getOutputCurrent, rightLeader::getOutputCurrent },
        (values) -> inputs.rightCurrentAmps = values);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    // Set PID targets
    leftPIDController.setReference(
        leftRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
    rightPIDController.setReference(
        rightRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFFVolts);
  }
}
