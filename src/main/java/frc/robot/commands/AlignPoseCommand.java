package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignPoseCommand extends Command {
    private final HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(
            AlignmentConstants.TRANSLATIONAL_PID_KP,
            AlignmentConstants.TRANSLATIONAL_PID_KI,
            AlignmentConstants.TRANSLATIONAL_PID_KD
        ),
        new PIDController(
            AlignmentConstants.TRANSLATIONAL_PID_KP,
            AlignmentConstants.TRANSLATIONAL_PID_KI,
            AlignmentConstants.TRANSLATIONAL_PID_KD
        ),
        new ProfiledPIDController(
            AlignmentConstants.ROTATIONAL_PID_KP,
            AlignmentConstants.ROTATIONAL_PID_KI,
            AlignmentConstants.ROTATIONAL_PID_KD,
            new TrapezoidProfile.Constraints(
                AlignmentConstants.MAXIMUM_SPEED_RADIANS,
                AlignmentConstants.MAXIMUM_ACCELERATION_RADIANS
            )
        )
    );

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Pose2d> target;

    public AlignPoseCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;

        controller.setTolerance(AlignmentConstants.TOLERANCE);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        controller.getXController().reset();
        controller.getYController().reset();
        controller.getThetaController().reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = controller.calculate(current, target.get(), 0.0, target.get().getRotation());
        swerveSubsystem.drive(chassisSpeeds);
    }
}
