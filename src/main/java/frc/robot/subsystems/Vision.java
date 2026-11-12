package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Feeds AprilTag pose estimates from one or more Limelights into the drivetrain's
 * pose estimator. This subsystem never drives the robot; it only corrects where the
 * drivetrain thinks it is on the field.
 *
 * See LIMELIGHT.md in the project root for how this fits together.
 */
public class Vision extends SubsystemBase {
    /* MegaTag2 is unreliable while spinning fast, because the camera frame and the gyro reading drift apart. */
    private static final double kMaxYawRateDegPerSec = 720;

    /*
     * How much to trust vision, as standard deviations in [x meters, y meters, heading radians].
     * Bigger numbers mean less trust. Heading is effectively ignored for MegaTag2 because
     * MegaTag2 takes its heading from our gyro, so it cannot correct it.
     */
    private static final double kMegaTag2XYStdDev = 0.7;
    private static final double kIgnoreStdDev = 9999999;

    /* While disabled, MegaTag1 with 2+ tags is trusted for heading so the gyro can be seeded before a match. */
    private static final double kMegaTag1XYStdDev = 0.5;
    private static final double kMegaTag1HeadingStdDev = Units.degreesToRadians(5);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final String[] m_cameraNames;
    private final StructPublisher<Pose2d>[] m_posePublishers;

    /**
     * @param drivetrain  Drivetrain whose pose estimator receives the measurements
     * @param cameraNames Limelight hostnames as configured in each camera's web UI (e.g. "limelight-front")
     */
    @SuppressWarnings("unchecked")
    public Vision(CommandSwerveDrivetrain drivetrain, String... cameraNames) {
        m_drivetrain = drivetrain;
        m_cameraNames = cameraNames;
        m_posePublishers = new StructPublisher[cameraNames.length];
        var table = NetworkTableInstance.getDefault().getTable("Vision");
        for (int i = 0; i < cameraNames.length; ++i) {
            m_posePublishers[i] = table.getStructTopic(cameraNames[i] + "/Pose", Pose2d.struct).publish();
        }
    }

    @Override
    public void periodic() {
        var state = m_drivetrain.getState();
        double headingDeg = state.Pose.getRotation().getDegrees();
        double yawRateDegPerSec = Units.radiansToDegrees(state.Speeds.omegaRadiansPerSecond);

        for (int i = 0; i < m_cameraNames.length; ++i) {
            String name = m_cameraNames[i];

            if (DriverStation.isDisabled()) {
                seedHeadingFromMegaTag1(name, i);
                continue;
            }

            /* MegaTag2 needs our field-relative heading every loop to solve for position. */
            LimelightHelpers.SetRobotOrientation(name, headingDeg, 0, 0, 0, 0, 0);
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

            if (estimate == null || estimate.tagCount == 0 || Math.abs(yawRateDegPerSec) > kMaxYawRateDegPerSec) {
                continue;
            }

            m_posePublishers[i].set(estimate.pose);
            m_drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(kMegaTag2XYStdDev, kMegaTag2XYStdDev, kIgnoreStdDev)
            );
        }
    }

    /**
     * MegaTag1 solves for the full pose (including heading) from tags alone, so it can
     * correct a gyro that was zeroed facing the wrong way. It's noisy with a single tag,
     * so we only use it when it sees at least two.
     */
    private void seedHeadingFromMegaTag1(String name, int index) {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (estimate == null || estimate.tagCount < 2) {
            return;
        }

        m_posePublishers[index].set(estimate.pose);
        m_drivetrain.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VecBuilder.fill(kMegaTag1XYStdDev, kMegaTag1XYStdDev, kMegaTag1HeadingStdDev)
        );
    }
}
