package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PiConstants;

/**
 * AprilTagSubsystem handles April tag detection and pose estimation using
 * PhotonVision.
 * This subsystem provides methods to get detected April tags and their
 * positions.
 */
public class AprilTagSubsystem extends SubsystemBase {

  private static AprilTagSubsystem self;

  private final PhotonCamera[] m_cameras;
  private final List<PhotonPipelineResult> m_unreadResults = new ArrayList<>();

  private final Transform3d m_cameraToRobot = new Transform3d();

  private PhotonPipelineResult m_latestResult;
  private List<PhotonTrackedTarget> m_latestTargets = new ArrayList<>();

  public static class AprilTagData {
    public final int id;
    public final Pose3d pose3d;
    public final Pose2d pose2d;
    public final double distance;
    public final double yaw;
    public final double pitch;
    public final double area;

    public AprilTagData(int id, Transform3d pose, double distance, double yaw, double pitch, double area) {
      this.id = id;
      this.pose3d = new Pose3d(pose.getTranslation(), pose.getRotation());
      this.pose2d = new Pose2d(pose.getTranslation().toTranslation2d(), pose.getRotation().toRotation2d());
      this.distance = distance;
      this.yaw = yaw;
      this.pitch = pitch;
      this.area = area;
    }
  }

  public static AprilTagSubsystem GetInstance() {
    if (self == null) {
      self = new AprilTagSubsystem();
    }
    return self;
  }

  public static List<AprilTagData> GetAllTags() {
    return self.getDetectedAprilTags();
  }

  public static Optional<AprilTagData> GetBestTag() {
    return self.getBestAprilTag();
  }

  public static Optional<AprilTagData> GetTagById(int id) {
    return self.getAprilTagById(id);
  }

  private AprilTagSubsystem() {
    m_cameras = PiConstants.CameraConstants.photonCamerasInUse;
  }

  public List<AprilTagData> getDetectedAprilTags() {
    List<AprilTagData> aprilTagData = new ArrayList<>();

    for (PhotonTrackedTarget target : m_latestTargets) {
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      aprilTagData.add(new AprilTagData(
          target.getFiducialId(),
          target.getBestCameraToTarget(),
          distance,
          target.getYaw(),
          target.getPitch(),
          target.getArea()));
    }

    return aprilTagData;
  }

  public Optional<AprilTagData> getBestAprilTag() {
    List<AprilTagData> tags = getDetectedAprilTags();

    if (tags.isEmpty()) {
      return Optional.empty();
    }

    return tags.stream()
        .max((a, b) -> Double.compare(a.area, b.area));
  }

  public Optional<AprilTagData> getAprilTagById(int id) {
    return getDetectedAprilTags().stream()
        .filter(tag -> tag.id == id)
        .findFirst();
  }

  public int getDetectedTagCount() {
    return m_latestTargets.size();
  }

  public boolean hasTargets() {
    return m_latestResult != null && m_latestResult.hasTargets();
  }

  public Transform3d getCameraToRobotTransform() {
    return m_cameraToRobot;
  }

  @Override
  public void periodic() {
    // Clear previous unread results
    m_unreadResults.clear();
    m_latestTargets.clear();

    // Process all unread results from all cameras
    for (PhotonCamera camera : m_cameras) {
      List<PhotonPipelineResult> cameraResults = camera.getAllUnreadResults();
      m_unreadResults.addAll(cameraResults);

      // Process each result from this camera
      for (PhotonPipelineResult result : cameraResults) {
        if (result.hasTargets()) {
          m_latestTargets.addAll(result.getTargets());
        }
      }
    }

    // Use the most recent result for compatibility
    if (!m_unreadResults.isEmpty()) {
      m_latestResult = m_unreadResults.get(m_unreadResults.size() - 1);
    } else {
      m_latestResult = null;
    }

    Logger.recordOutput("AprilTag/DetectedTargets", m_latestTargets.size());
    Logger.recordOutput("AprilTag/HasTargets", m_latestResult != null && m_latestResult.hasTargets());
    Logger.recordOutput("AprilTag/UnreadResultsCount", m_unreadResults.size());

    for (int i = 0; i < m_latestTargets.size(); i++) {
      PhotonTrackedTarget target = m_latestTargets.get(i);
      Logger.recordOutput("AprilTag/Target" + i + "/ID", target.getFiducialId());
      Logger.recordOutput("AprilTag/Target" + i + "/Yaw", target.getYaw());
      Logger.recordOutput("AprilTag/Target" + i + "/Pitch", target.getPitch());
      Logger.recordOutput("AprilTag/Target" + i + "/Area", target.getArea());
      Logger.recordOutput("AprilTag/Target" + i + "/Pose/3d", target.getBestCameraToTarget());
      Logger.recordOutput("AprilTag/Target" + i + "/Pose/2d",
          target.getBestCameraToTarget().getTranslation().toTranslation2d());
    }
  }
}
