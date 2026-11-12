package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Drives the robot to a field pose (blue-origin coordinates) using the drivetrain's
 * fused odometry + vision pose. Ends once the robot is within tolerance of the target.
 *
 * Each axis (x, y, heading) has its own motion-profiled PID controller: the profile
 * limits speed and acceleration, and PID corrects the remaining error.
 */
public class DriveToPose extends Command {
    /* Conservative limits for a first test; raise once the robot behaves. */
    private static final double kMaxSpeedMetersPerSec = 1.0;
    private static final double kMaxAccelMetersPerSecSq = 1.5;
    private static final double kMaxAngularSpeedRadPerSec = Math.PI;
    private static final double kMaxAngularAccelRadPerSecSq = 2 * Math.PI;

    /* PID gains: output in m/s (or rad/s) per meter (or radian) of error. Tune on the robot. */
    private static final double kTranslationP = 3.0;
    private static final double kHeadingP = 4.0;

    private static final double kPositionToleranceMeters = 0.02;
    private static final double kHeadingToleranceRad = Units.degreesToRadians(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Supplier<Pose2d> m_targetSupplier;
    private Pose2d m_target = Pose2d.kZero;

    private final ProfiledPIDController m_xController = new ProfiledPIDController(
        kTranslationP, 0, 0, new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSec, kMaxAccelMetersPerSecSq));
    private final ProfiledPIDController m_yController = new ProfiledPIDController(
        kTranslationP, 0, 0, new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSec, kMaxAccelMetersPerSecSq));
    private final ProfiledPIDController m_headingController = new ProfiledPIDController(
        kHeadingP, 0, 0, new TrapezoidProfile.Constraints(kMaxAngularSpeedRadPerSec, kMaxAngularAccelRadPerSecSq));

    /* Target pose is always in blue-origin field coordinates, so ignore the driver's alliance perspective. */
    private final SwerveRequest.FieldCentric m_request = new SwerveRequest.FieldCentric()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity);

    private final StructPublisher<Pose2d> m_targetPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("DriveToPose/Target", Pose2d.struct).publish();

    /**
     * @param drivetrain     Drivetrain to control
     * @param targetSupplier Target pose, read once when the command starts
     */
    public DriveToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
        m_drivetrain = drivetrain;
        m_targetSupplier = targetSupplier;

        m_xController.setTolerance(kPositionToleranceMeters);
        m_yController.setTolerance(kPositionToleranceMeters);
        m_headingController.setTolerance(kHeadingToleranceRad);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_target = m_targetSupplier.get();
        m_targetPublisher.set(m_target);

        /* Start each profile from where the robot is and how fast it's already moving, so there's no jerk. */
        var state = m_drivetrain.getState();
        Pose2d pose = state.Pose;
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, pose.getRotation());

        m_xController.reset(pose.getX(), fieldSpeeds.vxMetersPerSecond);
        m_yController.reset(pose.getY(), fieldSpeeds.vyMetersPerSecond);
        m_headingController.reset(pose.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond);

        m_xController.setGoal(m_target.getX());
        m_yController.setGoal(m_target.getY());
        m_headingController.setGoal(m_target.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivetrain.getState().Pose;

        /* PID correction plus the profile's planned velocity (feedforward). */
        double vx = m_xController.calculate(pose.getX()) + m_xController.getSetpoint().velocity;
        double vy = m_yController.calculate(pose.getY()) + m_yController.getSetpoint().velocity;
        double omega = m_headingController.calculate(pose.getRotation().getRadians())
            + m_headingController.getSetpoint().velocity;

        m_drivetrain.setControl(m_request.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    }

    @Override
    public boolean isFinished() {
        return m_xController.atGoal() && m_yController.atGoal() && m_headingController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
