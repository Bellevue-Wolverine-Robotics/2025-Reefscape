package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.utils.PIDUtils;
import com.revrobotics.spark.SparkMax;

public class IntakeSubsystem extends SubsystemBase {
	private SparkMax m_intakeArm;
	private RelativeEncoder m_intakeArmRelativeEncoder;
	private SparkPIDController m_intakeArmPID;

	private WPI_TalonSRX m_feederMotor;
	private DigitalInput limitSwitch = new DigitalInput(IntakeConstants.kNoteSwitchDIOPort);

	public IntakeSubsystem() {
		m_feederMotor = new WPI_TalonSRX(IntakeConstants.kFeederMotorId);
		m_feederMotor.configFactoryDefault();
		m_feederMotor.configPeakCurrentLimit(20);

		m_intakeArm = new CANSparkMax(IntakeConstants.kArmMotorId, MotorType.kBrushless);
		m_intakeArm.restoreFactoryDefaults();

		m_intakeArm.setInverted(true);
		m_intakeArm.setIdleMode(IdleMode.kBrake);

		m_feederMotor.setNeutralMode(NeutralMode.Coast);

		m_intakeArmRelativeEncoder = m_intakeArm.getEncoder();
		m_intakeArmRelativeEncoder.setPosition(0);

		m_intakeArmPID = m_intakeArm.getPIDController();

		PIDUtils.setPIDConstants(m_intakeArm.getPIDController(), IntakeConstants.kIntakeArmDeployPIDParams);

		SmartDashboard.putNumber("m_intakeArmPID kP", m_intakeArmPID.getP());
		SmartDashboard.putNumber("m_intakeArmPID kI", m_intakeArmPID.getI());
		SmartDashboard.putNumber("m_intakeArmPID kD", m_intakeArmPID.getD());
		SmartDashboard.putNumber("m_intakeArmPID kD", m_intakeArmPID.getFF());
	}

	// -- Intake Arm -- //
	public double getAngle() {
		return m_intakeArmRelativeEncoder.getPosition();
	}

	public void setIntakeArmPIDSetpoint(double setpoint) {
		//System.out.println("SET SETPOINT: " + setpoint);
		m_intakeArmPID.setReference(setpoint, ControlType.kPosition);
	}

	public void deployIntakeArm() {
		PIDUtils.setPIDConstants(m_intakeArm.getPIDController(), IntakeConstants.kIntakeArmDeployPIDParams);

		setIntakeArmPIDSetpoint(IntakeConstants.kIntakeArmSetpointDeploy);

	}

	public void stowIntakeArm() {
		PIDUtils.setPIDConstants(m_intakeArm.getPIDController(), IntakeConstants.kIntakeArmStowPIDParams);

		setIntakeArmPIDSetpoint(IntakeConstants.kIntakeArmSetpointStow);
	}

	public boolean isIntakeArmStowed() {
		return Math.abs(getAngle() - IntakeConstants.kIntakeArmSetpointStow) < IntakeConstants.kIntakeArmStowTolerance;

	}

	// -- Intake Motor -- //
	public void setIntakeMotorSpeed(double speed) {
		if (!limitSwitch.get()) {
			//System.out.println("AHHHHHiufh8ey8eiowehuef fucifkm");
			m_feederMotor.set(speed);
		}
	}

	public void startIntakeMotor() {
		setIntakeMotorSpeed(IntakeConstants.kFeederDutyCycle);
	}

	public int ahhhTestingIntakeDeleteMe = 1;
	public void stopIntakeMotor() {
		//System.out.println("STOPPPP stopIntakeMotor: Intake MOtor " + ahhhTestingIntakeDeleteMe);
		ahhhTestingIntakeDeleteMe ++;
		m_feederMotor.stopMotor();
	}

	public void startFeedIntoFlywheel() {
		setIntakeMotorSpeed(IntakeConstants.kFeedIntoFlywheelDutyCycle);
	}

	public boolean hasNote() {
		return limitSwitch.get();
	}

	@Override
	public void periodic() {

		// var intakeArmParams = new PIDUtils.SparkPIDParams(m_intakeArm);
		// intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kP",
		// m_intakeArmPID.getP()));
		// intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kI",
		// m_intakeArmPID.getI()));
		// intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kD",
		// m_intakeArmPID.getD()));
		// intakeArmParams.changeKp(SmartDashboard.getNumber("m_intakeArmPID kD",
		// m_intakeArmPID.getFF()));

		// PIDUtils.setPIDConstants(m_intakeArmPID, intakeArmParams);

		/*if (limitSwitch.get()) {
			//System.out.println("limit switch status: " + limitSwitch.get());
			//m_feederMotor.stopMotor();
			stopIntakeMotor();
		}*/

		SmartDashboard.putNumber("ARM POSITION", getAngle());
	}

	// DEBUG STUFF
	public void setFeederMotor(CommandJoystick joystick) {
		System.out.println("JOYSTICK AT: " + -joystick.getY());
		m_feederMotor.set(-joystick.getY());
	}

	public void setArmMotor(CommandJoystick joystick) {
		System.out.println("JOYSTICK AT: " + -joystick.getY());
		m_intakeArm.set(-joystick.getY() / 5);
	}

	// -- SHOOTER MODE -- //
	public void shoot() {
		setIntakeMotorSpeed(-1.0);
	}

}