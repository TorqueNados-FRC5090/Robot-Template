package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * Feeds every Limelight's AprilTag pose into the drivetrain's pose estimator.
 * <p>
 * Enabled: MegaTag2. It takes the heading from the Pigeon and only corrects X/Y, which keeps the
 * pose steady (MegaTag1 single-tag headings jump around).
 * <p>
 * Disabled: MegaTag1 with 2+ tags is allowed to fix the heading too, so MegaTag2 starts the match
 * with a correct heading even if the robot was placed crooked.
 * <p>
 * Cameras are listed in {@code Constants.VisionConstants.kCameraNames}. Each Limelight also needs its
 * camera position set on its web page (Settings, "Robot Space Pose") or the poses will be wrong.
 */
public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private double m_lastTagTime = 0.0;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        // Dashboard switch: false = ignore the cameras completely (pure wheel odometry)
        SmartDashboard.putBoolean("Vision/Use camera", true);
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean("Vision/Use camera", true)) {
            for (String camera : kCameraNames) {
                updateCamera(camera);
            }
        }
        SmartDashboard.putNumber("Vision/Seconds since last tag", Timer.getFPGATimestamp() - m_lastTagTime);
    }

    /** @return Whether any camera has given a pose in the last {@code seconds} seconds. */
    public boolean seenTagRecently(double seconds) {
        return Timer.getFPGATimestamp() - m_lastTagTime < seconds;
    }

    private void updateCamera(String camera) {
        var driveState = m_drivetrain.getState();

        // MegaTag2 needs the robot's heading every loop, so send it even if we skip this frame
        LimelightHelpers.SetRobotOrientation(camera, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // Vision is unreliable while spinning fast
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        if (Math.abs(omegaRps) >= kMaxSpinRps) {
            return;
        }

        if (DriverStation.isDisabled()) {
            PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);
            if (mt1 != null && mt1.tagCount >= kMegaTag1MinTags) {
                recordCorrection(camera, driveState.Pose, mt1);
                m_drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds,
                    VecBuilder.fill(kMegaTag1StdDev, kMegaTag1StdDev, kMegaTag1StdDev));
            }
        } else {
            PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
            if (mt2 != null && mt2.tagCount > 0) {
                recordCorrection(camera, driveState.Pose, mt2);
                // Huge heading std dev = ignore the vision heading, trust the Pigeon
                m_drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
                    VecBuilder.fill(kMegaTag2XYStdDev, kMegaTag2XYStdDev, 9999999));
            }
        }
    }

    /** How far the camera disagrees with odometry each time it sees tags. Small (under ~0.3m) = odometry is
     *  tracking well on its own. Big = odometry drifts when it can't see tags (wheel size, gearing, or gyro). */
    private void recordCorrection(String camera, Pose2d odometryPose, PoseEstimate estimate) {
        m_lastTagTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("Vision/" + camera + "/Tags seen", estimate.tagCount);
        SmartDashboard.putNumber("Vision/" + camera + "/Correction (m)",
            odometryPose.getTranslation().getDistance(estimate.pose.getTranslation()));
        SmartDashboard.putNumber("Vision/" + camera + "/Heading disagreement (deg)",
            odometryPose.getRotation().minus(estimate.pose.getRotation()).getDegrees());
    }
}
