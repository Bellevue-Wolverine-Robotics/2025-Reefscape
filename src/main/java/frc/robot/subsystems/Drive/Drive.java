package frc.robot.subsystems.Drive;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.DriveConstants;
import frc.utils.PIDUtils;

public class Drive extends SubsystemBase {
    private SparkMax m_backLeftMotor = new SparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
    private SparkMax m_frontLeftMotor = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
    private SparkMax m_frontRightMotor = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
    private SparkMax m_backRightMotor = new SparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

    private DifferentialDrive m_drive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);

    private RelativeEncoder m_leftEncoder = m_frontLeftMotor.getEncoder();
    private RelativeEncoder m_rightEncoder = m_frontRightMotor.getEncoder();

    private DifferentialDriveOdometry m_odometry;

    private SparkClosedLoopController m_frontLeftPID;
    private SparkClosedLoopController m_frontRightPID;
    private SparkClosedLoopController m_backLeftPID;
    private SparkClosedLoopController m_backRightPID;

    // fix AHRS later
    private AHRS m_imu = new AHRS(SPI.Port.kMXP);
    // private Debug debugLogger;
    // debug not needed for sim?

    private int throttleMode = 1;

    private Field2d m_field = new Field2d();
    // private VisionSubsystem vision = new VisionSubsystem();// NESTED
    // SUBSYSTEMS????
    // private Pose2d currPose2d;


    private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
            new DifferentialDriveKinematics(0.0),
            m_imu.getRotation2d(),
            m_leftEncoder.getPosition(),
            m_rightEncoder.getPosition(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // TODO: update these to reflect actual values using SysId
    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
            7.29, // 7.29:1 gearing reduction.
            7.5, // MOI of 7.5 kg m^2 (from CAD model).
            60.0, // The mass of the robot is 60 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            0.7112, // The track width is 0.7112 meters.

            // The standard deviations for measurement noise:
            // x and y: 0.001 m
            // heading: 0.001 rad
            // l and r velocity: 0.1 m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    public Drive(Debug debugLogger) {
        SmartDashboard.putData("Field", m_field);

        this.m_frontLeftMotor.restoreFactoryDefaults();
        this.m_backLeftMotor.restoreFactoryDefaults();
        this.m_frontRightMotor.restoreFactoryDefaults();
        this.m_backRightMotor.restoreFactoryDefaults();

        m_frontLeftMotor.setSmartCurrentLimit(30);
        m_frontRightMotor.setSmartCurrentLimit(30);
        m_backLeftMotor.setSmartCurrentLimit(30);
        m_backRightMotor.setSmartCurrentLimit(30);

        m_frontLeftMotor.setIdleMode(IdleMode.kBrake); 
        m_backLeftMotor.setIdleMode(IdleMode.kBrake);
        m_frontRightMotor.setIdleMode(IdleMode.kBrake);
        m_backRightMotor.setIdleMode(IdleMode.kBrake);

        m_backLeftMotor.follow(m_frontLeftMotor);
        m_backRightMotor.follow(m_frontRightMotor);

        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

        m_leftEncoder.setPositionConversionFactor(
                DriveConstants.kWheelCircumferenceMeters / DriveConstants.kDriveGearRatio);
        m_rightEncoder.setPositionConversionFactor(
                DriveConstants.kWheelCircumferenceMeters / DriveConstants.kDriveGearRatio);

        // WPILIB expects encoder rate to be in M/S while REV returns M/Min
        m_leftEncoder.setVelocityConversionFactor(
                (DriveConstants.kWheelCircumferenceMeters / DriveConstants.kDriveGearRatio) / 60);
        m_rightEncoder.setVelocityConversionFactor(
                (DriveConstants.kWheelCircumferenceMeters / DriveConstants.kDriveGearRatio) / 60);

        m_frontRightMotor.setInverted(true);

        m_imu.reset();
        m_imu.resetDisplacement();
        m_imu.zeroYaw();

        m_odometry = new DifferentialDriveOdometry(m_imu.getRotation2d(), m_leftEncoder.getPosition(),
                m_rightEncoder.getPosition());

        this.debugLogger = debugLogger;



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

        m_frontLeftPID = m_frontLeftMotor.getPIDController();
        m_frontRightPID = m_frontRightMotor.getPIDController();
        m_backLeftPID = m_backLeftMotor.getPIDController();
        m_backRightPID = m_backRightMotor.getPIDController();

        PIDUtils.setPIDConstants(m_frontLeftPID, DriveConstants.kDriveVelocityPIDParams);
        PIDUtils.setPIDConstants(m_frontRightPID, DriveConstants.kDriveVelocityPIDParams);
        PIDUtils.setPIDConstants(m_backLeftPID, DriveConstants.kDriveVelocityPIDParams);
        PIDUtils.setPIDConstants(m_backRightPID, DriveConstants.kDriveVelocityPIDParams);

        


        AutoBuilder.configureRamsete(
                this::getPose,
                this::resetPose,
                this::getCurrentSpeeds,
                this::drive,
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                bsupply::getAsBoolean, // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                this);

        // System.out.println("IF YOU FORGET THIS THEN YOU WILL BE ");
        // resetPose(new Pose2d(1.39, 5.5 3, new Rotation2d(2.34)));

        SmartDashboard.putNumber("Drive Kp Left", m_frontLeftPID.getP());
        SmartDashboard.putNumber("Drive Kp Right", m_frontRightPID.getP());
        SmartDashboard.putNumber("Drive Ki Left", m_frontLeftPID.getI());
        SmartDashboard.putNumber("Drive Ki Right", m_frontRightPID.getI());
        SmartDashboard.putNumber("Drive Kd Left", m_frontLeftPID.getD());
        SmartDashboard.putNumber("Drive Kd Right", m_frontRightPID.getD());
        SmartDashboard.putNumber("Drive Kf Left", m_frontLeftPID.getFF());
        SmartDashboard.putNumber("Drive Kf Right", m_frontRightPID.getFF());

    // }





    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
                pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidthMeters);

        var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());

        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

        return chassisSpeeds;
    }

    public void tankDrive(double left, double right) {
        this.m_drive.tankDrive(left, right);
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
                DriveConstants.kTrackWidthMeters);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;

        SmartDashboard.putNumber("Left Velocity Setpoint", leftVelocity);
        SmartDashboard.putNumber("Right Velocity Setpoint", rightVelocity);

        SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getVelocity());

        // System.out.println("Setpoints: " + leftVelocity + ", " + rightVelocity);
        // System.out.println("Velocity: " + m_leftEncoder.getVelocity() + ", " +
        // m_rightEncoder.getVelocity());

        // TODO: TEST IF WE NEED TO APPLY THESE TO THE BACK MOTORCONTROLLER PIDS TOO
        m_frontLeftPID.setReference(leftVelocity, ControlType.kVelocity);
        m_frontRightPID.setReference(rightVelocity, ControlType.kVelocity);

        m_drive.feed();
    }

    public void arcadeDrive(double xSpeed, double rotation) {
        this.m_drive.arcadeDrive(DriveConstants.driveThrottles[this.throttleMode].applyGradient(xSpeed), DriveConstants.driveThrottles[this.throttleMode].applyGradient(rotation));
    }


    public void setThrottle(ThrottlesSmartdashboard throttles) {

        switch(throttles){
            case FAST: {
              this.throttleMode = 0;
              break;
            }
            case MEDIUM: {
              this.throttleMode = 1;
              break;
            }
            case SLOW: {
              this.throttleMode = 2;
              break;
            }
            default:{
              this.throttleMode = 0;
              break;
            }
        }
        DriveConstants.driveThrottles[this.throttleMode].setLimit(m_frontLeftMotor);
        DriveConstants.driveThrottles[this.throttleMode].setLimit(m_frontRightMotor);
        DriveConstants.driveThrottles[this.throttleMode].setLimit(m_backLeftMotor);
        DriveConstants.driveThrottles[this.throttleMode].setLimit(m_backRightMotor);
    }

    public double getYaw() {
        return m_imu.getYaw();
    }

    public void stopDriveTrain() {
        m_drive.stopMotor();
    }

    @Override
    public void periodic() {


        if(DebugSettings.debugMode){
            SmartDashboard.putNumber("Current Y position: ", getPose().getY());
            SmartDashboard.putNumber("Current X position: ", getPose().getX());
            SmartDashboard.putNumber("Current Heading: ", getPose().getRotation().getDegrees());
            SmartDashboard.putNumber("m_imu: ", m_imu.getAngle());

            SmartDashboard.putNumber("Left Encoder: ", m_leftEncoder.getPosition());
            SmartDashboard.putNumber("Right Encoder: ", m_rightEncoder.getPosition());

            SmartDashboard.putNumber("Front Left Duty Cycle: ", m_frontLeftMotor.get());
            SmartDashboard.putNumber("Front Left Distance: ", m_leftEncoder.getPosition());
            SmartDashboard.putNumber("Front Left Rate: ", m_leftEncoder.getVelocity());
            SmartDashboard.putNumber("Back Left Rate: ", m_backLeftMotor.getEncoder().getVelocity());

            SmartDashboard.putNumber("Sim Y position: ", m_driveSim.getPose().getY());
            SmartDashboard.putNumber("Sim X position: ", m_driveSim.getPose().getX());

            m_odometry.update(m_imu.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
            m_field.setRobotPose(getPose());

            var leftMotorParams = new PIDUtils.SparkPIDParams(m_frontLeftMotor);
            var rightMotorParams = new PIDUtils.SparkPIDParams(m_frontLeftMotor);

            leftMotorParams.changeKp(SmartDashboard.getNumber("Drive Kp Left", m_frontLeftPID.getP()));
            rightMotorParams.changeKp(SmartDashboard.getNumber("Drive Kp Right",m_frontRightPID.getP()));
            leftMotorParams.changeKi(SmartDashboard.getNumber("Drive Ki Left",m_frontLeftPID.getI()));
            rightMotorParams.changeKi(SmartDashboard.getNumber("Drive Ki Right",m_frontRightPID.getI()));
            leftMotorParams.changeKd(SmartDashboard.getNumber("Drive Kd Left", m_frontLeftPID.getD()));
            rightMotorParams.changeKd(SmartDashboard.getNumber("Drive Kd Right", m_frontRightPID.getD()));
            leftMotorParams.changeKff(SmartDashboard.getNumber("Drive Kf Left", m_frontLeftPID.getFF()));
            rightMotorParams.changeKff(SmartDashboard.getNumber("Drive Kf Right", m_frontRightPID.getFF()));

            PIDUtils.setPIDConstants(m_frontLeftPID, leftMotorParams);
            PIDUtils.setPIDConstants(m_backLeftPID, leftMotorParams);

            PIDUtils.setPIDConstants(m_frontRightPID, rightMotorParams);
            PIDUtils.setPIDConstants(m_backRightPID, rightMotorParams);
        }
    }

    @Override
    public void simulationPeriodic() {
        m_driveSim.setInputs(m_frontLeftMotor.get() * DriveConstants.kMaxSimInputVoltage,
                m_frontRightMotor.get() * DriveConstants.kMaxSimInputVoltage);

        // System.out.println(m_frontLeftMotor.get());

        m_driveSim.update(0.02);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(-m_driveSim.getHeading().getDegrees());
    }

    /**
     * Returns CanSparkMax array in format of
     * [frontLeft, frontRight, backLeft, backRight]
     */
    public CANSparkMax[] getDriveMotorControllers() {
        return new CANSparkMax[] { m_frontLeftMotor, m_frontRightMotor, m_backLeftMotor, m_backRightMotor };

        // andrew i was trying to resolve a merge conflict here idk what this code does

        // vision.getEstimatedGlobalPose(getPose());
        // m_poseEstimator.addVisionMeasurement(vision.getPose2d(),
        // vision.getTimestampSeconds());
    }

    public Command pathfindToStartCommand() {
        Pose2d targetPose = new Pose2d(0, 8, Rotation2d.fromDegrees(0));

        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );

        return pathfindingCommand;
    }

    public double getFrontRightRate() {
        return m_rightEncoder.getVelocity();
    }
}