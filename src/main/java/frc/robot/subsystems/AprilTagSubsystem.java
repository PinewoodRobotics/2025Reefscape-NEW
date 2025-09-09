package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.PiConstants;
import frc.robot.util.CustomMath;
import frc.robot.util.TimedAprilTagData;

public class AprilTagSubsystem extends SubsystemBase {

  private static AprilTagSubsystem self;

  private final PiConstants.CameraConstants.CameraMount[] m_cameraMounts;

  private static final double TAG_STALE_SECONDS = 0.200;

  private final Map<Integer, TimedAprilTagData> m_trackedTags = new HashMap<>();

  public static AprilTagSubsystem GetInstance() {
    if (self == null) {
      self = new AprilTagSubsystem();
    }
    return self;
  }

  public static List<TimedAprilTagData> GetAllTags() {
    return self.getDetectedAprilTags();
  }

  public static TimedAprilTagData GetBestTag() {
    return self.getBestAprilTag();
  }

  public static TimedAprilTagData GetTagById(int id) {
    return self.getAprilTagById(id);
  }

  private AprilTagSubsystem() {
    m_cameraMounts = PiConstants.CameraConstants.photonCamerasInUse;
  }

  public List<TimedAprilTagData> getDetectedAprilTags() {
    return new ArrayList<>(m_trackedTags.values());
  }

  public TimedAprilTagData getBestAprilTag() {
    return getDetectedAprilTags().stream()
        .min(Comparator.comparingDouble(a -> a.getPose2d().getTranslation().getNorm()))
        .orElse(null);
  }

  public TimedAprilTagData getAprilTagById(int id) {
    return getDetectedAprilTags().stream()
        .filter(tag -> tag.getId() == id)
        .findFirst()
        .orElse(null);
  }

  public int getDetectedTagCount() {
    return m_trackedTags.size();
  }

  public boolean hasTargets() {
    return !m_trackedTags.isEmpty();
  }

  public TimedAprilTagData getClosestTag() {
    return getDetectedAprilTags().stream()
        .min(Comparator.comparingDouble(a -> a.getPose2d().getTranslation().getNorm()))
        .orElse(null);
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> latestResults = new ArrayList<>();

    for (PiConstants.CameraConstants.CameraMount mount : m_cameraMounts) {
      List<PhotonPipelineResult> results = mount.camera.getAllUnreadResults();
      if (results.isEmpty()) {
        continue;
      }

      PhotonPipelineResult latest = results.get(results.size() - 1);
      latestResults.add(latest);
      if (!latest.hasTargets()) {
        continue;
      }

      double frameTimestamp = latest.getTimestampSeconds();
      double nowFPGA = Timer.getFPGATimestamp();
      for (PhotonTrackedTarget target : latest.getTargets()) {
        int id = target.getFiducialId();
        TimedAprilTagData prev = m_trackedTags.get(id);
        if (prev == null) {
          m_trackedTags.put(id,
              TimedAprilTagData.fromDetection(target, mount.T_cameraInRobot, frameTimestamp, nowFPGA));
        } else {
          m_trackedTags.put(id, prev.updateFrom(target, mount.T_cameraInRobot, frameTimestamp, nowFPGA));
        }
      }
    }

    double now = Timer.getFPGATimestamp();
    m_trackedTags.entrySet().removeIf(e -> !e.getValue().isFresh(now, TAG_STALE_SECONDS));

    List<TimedAprilTagData> current = new ArrayList<>(m_trackedTags.values());

    Logger.recordOutput("AprilTag/DetectedTargets", current.size());
    Logger.recordOutput("AprilTag/HasTargets", !current.isEmpty());
    Logger.recordOutput("AprilTag/LatestResultsCount", latestResults.size());

    for (int i = 0; i < current.size(); i++) {
      TimedAprilTagData data = current.get(i);
      Logger.recordOutput("AprilTag/Target" + i + "/ID", data.getId());
      Logger.recordOutput("AprilTag/Target" + i + "/Pose2d", data.getPose2d());
      Logger.recordOutput("AprilTag/Target" + i + "/Translation2d", data.getPose2d().getTranslation());
      Logger.recordOutput("AprilTag/Target" + i + "/RotationDeg", data.getPose2d().getRotation().getDegrees());
    }
  }
}
