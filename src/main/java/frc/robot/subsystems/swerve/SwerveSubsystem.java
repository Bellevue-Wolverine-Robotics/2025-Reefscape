package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.constants.*;
import frc.robot.subsystems.vision.EstimatedPoseStruct;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.XboxControllerWrapper;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// =========================================
// Class Declaration
// =========================================
public class SwerveSubsystem extends SubsystemBase {

  // =========================================
  // Field Declarations
  // =========================================

  // Swerve Drive & Vision Related Fields
  private final SwerveDrive swerveDrive;
  private final AprilTagFieldLayout aprilTagFieldLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private final boolean visionDriveTest = false;
  private boolean enableVisionOdometry = true;
  private VisionSubsystem visionSubsystem;

  // NetworkTables Publishers
  StructPublisher<Pose2d> odometryPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("SwerveSubsystem/OdometryPose", Pose2d.struct)
    .publish();
  StructPublisher<Pose2d> simPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("SwerveSubsystem/SimPose", Pose2d.struct)
    .publish();
  StructArrayPublisher<SwerveModuleState> moduleStatePublisher =
    NetworkTableInstance.getDefault()
      .getStructArrayTopic(
        "SwerveSubsystem/SwerveModuleStates",
        SwerveModuleState.struct
      )
      .publish();

  // =========================================
  // Constructors & Initialization
  // =========================================

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory, VisionSubsystem visionSubsystem) {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
        DriveConstants.kMaxSpeedMetersPerSec,
        new Pose2d(
          new Translation2d(Meter.of(1), Meter.of(4)),
          Rotation2d.fromDegrees(0)
        )
      );
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.setChassisDiscretization(true, 0.05);
    swerveDrive.setAngularVelocityCompensation(
      true,
      true,
      DriveConstants.kAngularVelocityCompensation
    );

    // swerveDrive.stopOdometryThread();
    setupPathPlanner();

    this.visionSubsystem = visionSubsystem;
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(
    SwerveDriveConfiguration driveCfg,
    SwerveControllerConfiguration controllerCfg
  ) {
    swerveDrive = new SwerveDrive(
      driveCfg,
      controllerCfg,
      DriveConstants.kMaxSpeedMetersPerSec,
      new Pose2d(
        new Translation2d(Meter.of(2), Meter.of(0)),
        Rotation2d.fromDegrees(0)
      )
    );
  }

  // =========================================
  // Periodic Methods
  // =========================================

  @Override
  public void periodic() {
    // Publish your usual data...
    odometryPublisher.set(this.getPose());
    moduleStatePublisher.set(
      swerveDrive.kinematics.toSwerveModuleStates(getFieldVelocity())
    );

    // Only update odometry if vision data is available.
    // if (enableVisionOdometry) {
    //   Optional<EstimatedPoseStruct> estimatedPose =
    //     visionSubsystem.estimateRobotPoseVision();
    //   if (estimatedPose.isPresent()) {
    //     // Convert Transform3d to Pose2d.
    //     // Assuming the vision transform is planar, we can use a Pose3d conversion.
    //     Pose2d visionPose = estimatedPose.get().robotPose;
    //     swerveDrive.addVisionMeasurement(
    //       visionPose,
    //       estimatedPose.get().estimationTimestamp,
    //       VisionConstants.getEstimationStdDevs(estimatedPose.get().distance)
    //     );
    //   } else {}
    // }

    ArrayList<EstimatedPoseStruct> estimatedPoses =
      visionSubsystem.estimateRobotPoseFromAllCameras();

    for (EstimatedPoseStruct estimatedPose : estimatedPoses) {
      swerveDrive.addVisionMeasurement(
        estimatedPose.robotPose,
        estimatedPose.estimationTimestamp,
        VisionConstants.getEstimationStdDevs(estimatedPose.distance)
      );
    }
    // if (estimatedPoses) {
    //   // Convert Transform3d to Pose2d.
    //   // Assuming the vision transform is planar, we can use a Pose3d conversion.
    //   Pose2d visionPose = estimatedPose.get().robotPose;
    //   swerveDrive.addVisionMeasurement(
    //     visionPose,
    //     estimatedPose.get().estimationTimestamp,
    //     VisionConstants.getEstimationStdDevs(estimatedPose.get().distance)
    //   );
    // } else {}
    // }
  }

  @Override
  public void simulationPeriodic() {
    visionSubsystem.updateSim(swerveDrive.getSimulationDriveTrainPose().get());
    simPosePublisher.set(swerveDrive.getSimulationDriveTrainPose().get());
  }

  // =========================================
  // Setup & Configuration Methods
  // =========================================

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      AutoBuilder.configure(
        this::getPose,
        // Robot pose supplier
        this::resetOdometry,
        // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity,
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speedsRobotRelative, moduleFeedForwards) -> {
          if (enableFeedforward) {
            swerveDrive.drive(
              speedsRobotRelative,
              swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
              moduleFeedForwards.linearForces()
            );
          } else {
            swerveDrive.setChassisSpeeds(speedsRobotRelative);
          }
        },
        // PPHolonomicDriveController with PID constants for translation and rotation.
        new PPHolonomicDriveController(
          new PIDConstants(DriveConstants.PATH_PLANNER_LINEAR_KP, 0.0, 0.0),
          new PIDConstants(DriveConstants.PATH_PLANNER_ANGULAR_KP, 0.0, 0.0)
        ),
        config,
        // Supplier to mirror the path for red alliance if needed.
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent()
            ? alliance.get() == DriverStation.Alliance.Red
            : false;
        },
        this
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   * This version properly handles the "hold to drive" behavior with improved speed performance.
   *
   * @param poseSupplier Target {@link Supplier<Pose2d>} to go to.
   * @return PathFinding command that requires the subsystem until canceled
   */
  public Command driveToPose(Supplier<Pose2d> poseSupplier) {
    // Use an atomic reference to track the current pathfinding command
    AtomicReference<Command> currentPathCommand = new AtomicReference<>(null);
    return Commands.runOnce(() -> {
      System.out.println("Starting pathfinding command");
      // Create initial pathfinding command
      Pose2d pose = poseSupplier.get();
      System.out.println(pose);
      PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), // Use full speed
        5.5, // Maximum acceleration
        swerveDrive.getMaximumChassisAngularVelocity(), // Full angular velocity
        Units.degreesToRadians(720) // Maximum angular acceleration
      );
      Command pathfindCommand = AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // No speed reduction at end
      );
      // Store and schedule
      currentPathCommand.set(pathfindCommand);
      pathfindCommand.schedule();
    })
      .andThen(
        // This will only schedule a new command if the current one is complete
        Commands.run(() -> {
          Command current = currentPathCommand.get();
          if (current == null || current.isFinished()) {
            Pose2d pose = poseSupplier.get();
            PathConstraints constraints = new PathConstraints(
              swerveDrive.getMaximumChassisVelocity(),
              5.5,
              swerveDrive.getMaximumChassisAngularVelocity(),
              Units.degreesToRadians(720)
            );
            Command pathfindCommand = AutoBuilder.pathfindToPose(
              pose,
              constraints,
              edu.wpi.first.units.Units.MetersPerSecond.of(0)
            );
            currentPathCommand.set(pathfindCommand);
            pathfindCommand.schedule();
          }
        }).until(() -> false) // Run until interrupted
      )
      .finallyDo(interrupted -> {
        // This will run when the button is released
        System.out.println("COMMAND ENDED - Button released");
        // Cancel just the pathfinding command rather than all commands
        Command current = currentPathCommand.get();
        if (current != null) {
          current.cancel();
        }
      })
      .withTimeout(10); // Safety timeout in case something goes wrong
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
   * PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
   *                                  achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(
    Supplier<ChassisSpeeds> robotRelativeChassisSpeed
  ) throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
      RobotConfig.fromGUISettings(),
      swerveDrive.getMaximumChassisAngularVelocity()
    );
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
      new SwerveSetpoint(
        swerveDrive.getRobotVelocity(),
        swerveDrive.getStates(),
        DriveFeedforwards.zeros(swerveDrive.getModules().length)
      )
    );
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(
      () -> previousTime.set(Timer.getFPGATimestamp()),
      () -> {
        double newTime = Timer.getFPGATimestamp();
        SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
          prevSetpoint.get(),
          robotRelativeChassisSpeed.get(),
          newTime - previousTime.get()
        );
        swerveDrive.drive(
          newSetpoint.robotRelativeSpeeds(),
          newSetpoint.moduleStates(),
          newSetpoint.feedforwards().linearForces()
        );
        prevSetpoint.set(newSetpoint);
        previousTime.set(newTime);
      }
    );
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(
    Supplier<ChassisSpeeds> fieldRelativeSpeeds
  ) {
    try {
      return driveWithSetpointGenerator(() ->
        ChassisSpeeds.fromFieldRelativeSpeeds(
          fieldRelativeSpeeds.get(),
          getHeading()
        )
      );
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(
        new Config(),
        this,
        swerveDrive,
        12,
        true
      ),
      3.0,
      5.0,
      3.0
    );
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
      3.0,
      5.0,
      3.0
    );
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() ->
      Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0))
    );
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a
   * given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(
    double distanceInMeters,
    double speedInMetersPerSecond
  ) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0))
    ).until(
      () ->
        swerveDrive
          .getPose()
          .getTranslation()
          .getDistance(new Translation2d(0, 0)) >
        distanceInMeters
    );
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(
      new SimpleMotorFeedforward(kS, kV, kA)
    );
  }

  // =========================================
  // Drive & Control Commands
  // =========================================

  public Command driveAngularSpeedCommand(
    XboxControllerWrapper xboxController
  ) {
    return this.driveWithSetpointGeneratorFieldRelative(
        this.driveAngularSpeed(xboxController)
      );
    // return this.driveFieldOriented(this.driveAngularSpeed(xboxController));
  }

  /**
   * Creates a command to drive the robot with angular speed control and a speed reduction factor
   *
   * @param xboxController The Xbox controller to use for input
   * @param speedFactor Factor to reduce the speed by (0-1)
   * @return Command for reduced speed driving
   */
  public Command driveAngularSpeedCommand(
    XboxControllerWrapper xboxController,
    double speedFactor
  ) {
    return this.driveWithSetpointGeneratorFieldRelative(
        this.driveAngularSpeedReduced(xboxController, speedFactor)
      );
  }

  public Command driveDirectAngleCommand(XboxControllerWrapper xboxController) {
    return this.driveWithSetpointGeneratorFieldRelative(
        this.driveDirectAngle(xboxController)
      );
    // return this.driveFieldOriented(this.driveDirectAngle(xboxController));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier angularRotationX
  ) {
    return run(() -> {
      swerveDrive.drive(
        SwerveMath.scaleTranslation(
          new Translation2d(
            translationX.getAsDouble() *
            swerveDrive.getMaximumChassisVelocity(),
            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
          ),
          0.8
        ),
        Math.pow(angularRotationX.getAsDouble(), 3) *
        swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false
      );
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier headingX,
    DoubleSupplier headingY
  ) {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
        new Translation2d(
          translationX.getAsDouble(),
          translationY.getAsDouble()
        ),
        0.8
      );
      driveFieldOriented(
        swerveDrive.swerveController.getTargetSpeeds(
          scaledInputs.getX(),
          scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()
        )
      );
    });
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate,
   * and calculates and commands module states accordingly. Can use either open-loop or closed-loop velocity control
   * for the wheel velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per second.
   *                      In robot-relative mode, positive x is towards the bow (front) and positive y is towards port (left).
   *                      In field-relative mode, positive x is away from the alliance wall (field North) and positive y is
   *                      towards the left wall when looking through the driver station glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(
    Translation2d translation,
    double rotation,
    boolean fieldRelative
  ) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   * @return Command that drives field oriented.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  // =========================================
  // Utility & Getter Methods
  // =========================================

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent()
      ? alliance.get() == DriverStation.Alliance.Red
      : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drivebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(
        new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180))
      );
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction.
   * The other for the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
    double xInput,
    double yInput,
    double headingX,
    double headingY
  ) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(
      new Translation2d(xInput, yInput)
    );
    return swerveDrive.swerveController.getTargetSpeeds(
      scaledInputs.getX(),
      scaledInputs.getY(),
      headingX,
      headingY,
      getHeading().getRadians(),
      DriveConstants.kMaxSpeedMetersPerSec
    );
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
    double xInput,
    double yInput,
    Rotation2d angle
  ) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(
      new Translation2d(xInput, yInput)
    );
    return swerveDrive.swerveController.getTargetSpeeds(
      scaledInputs.getX(),
      scaledInputs.getY(),
      angle.getRadians(),
      getHeading().getRadians(),
      DriveConstants.kMaxSpeedMetersPerSec
    );
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} for the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(
      new Pose2d(3, 3, Rotation2d.fromDegrees(65)),
      Timer.getFPGATimestamp()
    );
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  // =========================================
  // Swerve Input Streams
  // =========================================

  public SwerveInputStream driveAngularSpeed(
    XboxControllerWrapper xboxController
  ) {
    return SwerveInputStream.of(
      this.getSwerveDrive(),
      () -> -xboxController.getLeftY(),
      () -> -xboxController.getLeftX()
    ).withControllerRotationAxis(() -> -xboxController.getRightX());
  }

  /**
   * Creates a SwerveInputStream with reduced speed for precision driving
   *
   * @param xboxController The Xbox controller to use for input
   * @param speedFactor The factor to reduce speed by (0-1)
   * @return SwerveInputStream with reduced speeds
   */
  public SwerveInputStream driveAngularSpeedReduced(
    XboxControllerWrapper xboxController,
    double speedFactor
  ) {
    System.out.println(-xboxController.getLeftY() * speedFactor);
    return SwerveInputStream.of(
      this.getSwerveDrive(),
      () -> -xboxController.getLeftY() * speedFactor,
      () -> -xboxController.getLeftX() * speedFactor
    ).withControllerRotationAxis(
      () -> -xboxController.getRightX() * speedFactor
    );
  }

  public SwerveInputStream driveDirectAngle(
    XboxControllerWrapper xboxController
  ) {
    return SwerveInputStream.of(
      this.getSwerveDrive(),
      () -> -xboxController.getLeftY(),
      () -> -xboxController.getLeftX()
    )
      .withControllerHeadingAxis(
        () -> -xboxController.getRightX(),
        () -> xboxController.getRightY()
      )
      .headingWhile(true)
      .allianceRelativeControl(true)
      .deadband(OperatorConstants.kDeadzone);
  }
}
