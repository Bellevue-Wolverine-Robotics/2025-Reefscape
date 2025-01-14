package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.constants.DriveConstants;

public class DriveSubystem {
    private SparkMax m_backLeftMotor = new SparkMax(DriveConstants.kBackLeftMotorId, SparkLowLevel.MotorType.kBrushless);
    private SparkMax m_frontLeftMotor = new SparkMax(DriveConstants.kFrontLeftMotorId, SparkLowLevel.MotorType.kBrushless);
    private SparkMax m_frontRightMotor = new SparkMax(DriveConstants.kFrontRightMotorId, SparkLowLevel.MotorType.kBrushless);
    private SparkMax m_backRightMotor = new SparkMax(DriveConstants.kBackRightMotorId, SparkLowLevel.MotorType.kBrushless);
    

    private DifferentialDrive m_drive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);

    private RelativeEncoder m_leftEncoder = m_frontLeftMotor.getEncoder();
    private RelativeEncoder m_rightEncoder = m_frontRightMotor.getEncoder();

    private DifferentialDriveOdometry m_odometry;

    private SparkClosedLoopController m_frontLeftPID;
    private SparkClosedLoopController m_frontRightPID;
    private SparkClosedLoopController m_backLeftPID;
    private SparkClosedLoopController m_backRightPID;

    


    public DriveSubystem() {
        configureMotors();
        configurePID();
    }


    private void configurePID() {
        m_frontLeftPID = m_frontLeftMotor.getClosedLoopController();
        m_frontRightPID = m_frontRightMotor.getClosedLoopController();
        m_backLeftPID = m_backLeftMotor.getClosedLoopController();
        m_backRightPID = m_backRightMotor.getClosedLoopController();
    }

    public SparkMax[] getDriveMotorControllers() {
        return new SparkMax[] { m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor };

        // andrew i was trying to resolve a merge conflict here idk what this code does

        // vision.getEstimatedGlobalPose(getPose());
        // m_poseEstimator.addVisionMeasurement(vision.getPose2d(),
        // vision.getTimestampSeconds());
    }
 
    private void configureMotors() {
        SparkBaseConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.smartCurrentLimit(30);
        rightConfig.follow(m_frontRightMotor);
        rightConfig.inverted(true);

        SparkBaseConfig leftConfig = new SparkMaxConfig();
        leftConfig.apply(rightConfig);
        leftConfig.follow(m_frontLeftMotor);
        leftConfig.inverted(false);

        this.m_frontLeftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.m_backLeftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.m_frontRightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.m_backRightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

        BooleanSupplier bsupply = (() -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        });
    }
}
