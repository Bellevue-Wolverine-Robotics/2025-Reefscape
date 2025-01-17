package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Mode;
import frc.robot.interfaces.DriveIO;
import frc.robot.interfaces.DriveIOInputsAutoLogged;
import frc.robot.interfaces.GyroIO;
import frc.robot.interfaces.GyroIOInputsAutoLogged;

/**
 * Class for the tank drivetrain subsystem
 */
public class DriveSubsystem extends SubsystemBase {
  // Drive motors instantiation
  private final DriveIO driveIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();

  // Gyro instantiation
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // set feed forward
  private final double kS = RobotConstants.currentMode == Mode.SIM ? DriveConstants.simKs : DriveConstants.realKs;
  private final double kV = RobotConstants.currentMode == Mode.SIM ? DriveConstants.simKv : DriveConstants.realKv;

  // heading
  private Rotation2d rawGyroRotation = new Rotation2d();

  // last encoder position
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;

  // pose estimator
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      DriveConstants.kTrackWidthMeters);
  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
      new Rotation2d(), 0.0, 0.0, new Pose2d());

  /**
   * Initialize an instantce of the drivetrain subsystem
   *
   * @param driveIO Drive motors class
   * @param gyroIO  Gyro class
   */
  public DriveSubsystem(DriveIO driveIO, GyroIO gyroIO) {
    this.driveIO = driveIO;
    this.gyroIO = gyroIO;

    // configure pathplanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
        (ChassisSpeeds speeds) -> runClosedLoop(speeds),
        new PPLTVController(0.02, DriveConstants.kMaxSpeedMetersPerSec),
        DriveConstants.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  @Override
  public void periodic() {
    // update state classes and log them
    driveIO.updateInputs(driveInputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", driveInputs);
    Logger.processInputs("Drive/Gyro", driveInputs);

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and encoder data
      Twist2d twist = kinematics.toTwist2d(
          getLeftPositionMeters() - lastLeftPositionMeters,
          getRightPositionMeters() - lastRightPositionMeters);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      lastLeftPositionMeters = getLeftPositionMeters();
      lastRightPositionMeters = getRightPositionMeters();
    }

    // Update odometry
    poseEstimator.update(rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds The speed at which to run the drivetrain at
   */
  public void runClosedLoop(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Runs the drive at the desired left and right velocities.
   *
   * @param leftMetersPerSec  The speed to run the left motors at
   * @param rightMetersPerSec The speed to run the right motors at
   *
   */
  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / DriveConstants.kWheelRadiusMeters;
    double rightRadPerSec = rightMetersPerSec / DriveConstants.kWheelRadiusMeters;
    Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftRadPerSec);
    Logger.recordOutput("Drive/RightSetpointRadPerSec", rightRadPerSec);

    double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
    double rightFFVolts = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;
    driveIO.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
  }

  /**
   * Runs the drive in open loop.
   *
   * @param leftVolts  Amount of voltage supplied to left motors
   * @param rightVolts Amount of voltage supplied to right motors
   */
  public void runOpenLoop(double leftVolts, double rightVolts) {
    driveIO.setVoltage(leftVolts, rightVolts);
  }

  /** Stops the drive. */
  public void stop() {
    runOpenLoop(0.0, 0.0);
  }

  /**
   * Returns the current odometry pose.
   *
   * @return Returns the current odometry pose
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current odometry pose.
   *
   * @param pose The new pose
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(
        rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  /**
   * Returns the current odometry rotation.
   *
   * @return Returns the current odometry rotation
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Returns the position of the left wheels in meters.
   *
   * @return Returns the position of the left wheels in meters
   */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return driveInputs.leftPositionRad * DriveConstants.kWheelDiameterInches;
  }

  /**
   * Returns the position of the right wheels in meters.
   *
   * @return Returns the position of the right wheels in meters
   */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return driveInputs.rightPositionRad * DriveConstants.kWheelDiameterInches;
  }

  /**
   * Returns the velocity of the left wheels in meters/second.
   *
   * @return Returns the velocity of the left wheels in meters/second
   */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return driveInputs.leftVelocityRadPerSec * DriveConstants.kWheelRadiusMeters;
  }

  /**
   * Returns the velocity of the right wheels in meters/second.
   *
   * @return Returns the velocity of the right wheels in meters/second
   */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return driveInputs.rightVelocityRadPerSec * DriveConstants.kWheelRadiusMeters;
  }

}
