package frc.robot.subsystems.camera;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.PiConstants;
import frc.robot.util.TimedRobotCentricAprilTagData;

public class AprilTagSubsystem extends SubsystemBase {

  private static AprilTagSubsystem self;

  private final CameraSystem[] m_cameraSystems;

  private static final double TAG_STALE_SECONDS = 0.200;

  private static final boolean AVERAGE_VALUES = true;

  private final Map<Integer, TimedRobotCentricAprilTagData> m_trackedTags = new HashMap<>();

  public static AprilTagSubsystem GetInstance() {
    if (self == null) {
      self = new AprilTagSubsystem();
    }
    return self;
  }

  public static List<TimedRobotCentricAprilTagData> GetAllTags() {
    return self.getDetectedAprilTags();
  }

  public static TimedRobotCentricAprilTagData GetBestTag() {
    return self.getBestAprilTag();
  }

  public static TimedRobotCentricAprilTagData GetTagById(int id) {
    return self.getAprilTagById(id);
  }

  private AprilTagSubsystem() {
    m_cameraSystems = PiConstants.CameraConstants.photonCamerasInUse;
  }

  public List<TimedRobotCentricAprilTagData> getDetectedAprilTags() {
    return new ArrayList<>(m_trackedTags.values());
  }

  public TimedRobotCentricAprilTagData getBestAprilTag() {
    return getDetectedAprilTags().stream()
        .min(Comparator.comparingDouble(a -> a.getPose2d().getTranslation().getNorm()))
        .orElse(null);
  }

  public TimedRobotCentricAprilTagData getAprilTagById(int id) {
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

  public TimedRobotCentricAprilTagData getClosestTag() {
    return getDetectedAprilTags().stream()
        .min(Comparator.comparingDouble(a -> a.getPose2d().getTranslation().getNorm()))
        .orElse(null);
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> latestResults = new ArrayList<>();
    Map<Integer, List<TimedRobotCentricAprilTagData>> tagDetectionsPerCamera = new HashMap<>();

    for (CameraSystem mount : m_cameraSystems) {
      List<PhotonPipelineResult> results = mount.getAllUnreadResults();
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
        TimedRobotCentricAprilTagData tagData = new TimedRobotCentricAprilTagData(target, frameTimestamp, nowFPGA,
            mount);

        if (AVERAGE_VALUES) {
          // Collect detections from multiple cameras for averaging
          tagDetectionsPerCamera.computeIfAbsent(id, k -> new ArrayList<>()).add(tagData);
        } else {
          // Use the old behavior - just overwrite with latest detection
          m_trackedTags.put(id, tagData);
        }
      }
    }

    // If averaging is enabled, process collected detections
    if (AVERAGE_VALUES) {
      for (Map.Entry<Integer, List<TimedRobotCentricAprilTagData>> entry : tagDetectionsPerCamera.entrySet()) {
        int tagId = entry.getKey();
        List<TimedRobotCentricAprilTagData> detections = entry.getValue();

        if (detections.size() == 1) {
          // Only one detection, no need to average
          m_trackedTags.put(tagId, detections.get(0));
        } else {
          // Multiple detections from different cameras, average them
          TimedRobotCentricAprilTagData averagedData = TimedRobotCentricAprilTagData
              .average(detections.toArray(new TimedRobotCentricAprilTagData[0]));
          m_trackedTags.put(tagId, averagedData);
        }
      }
    }

    double now = Timer.getFPGATimestamp();
    m_trackedTags.entrySet().removeIf(e -> !e.getValue().isFresh(now, TAG_STALE_SECONDS));

    List<TimedRobotCentricAprilTagData> current = new ArrayList<>(m_trackedTags.values());

    Logger.recordOutput("AprilTag/DetectedTargets", current.size());
    Logger.recordOutput("AprilTag/HasTargets", !current.isEmpty());
    Logger.recordOutput("AprilTag/LatestResultsCount", latestResults.size());
    Logger.recordOutput("AprilTag/AveragingEnabled", AVERAGE_VALUES);

    for (int i = 0; i < current.size(); i++) {
      TimedRobotCentricAprilTagData data = current.get(i);
      Logger.recordOutput("AprilTag/Target" + i + "/ID", data.getId());
      Logger.recordOutput("AprilTag/Target" + i + "/Pose2d", data.getPose2d());
      Logger.recordOutput("AprilTag/Target" + i + "/Translation2d", data.getPose2d().getTranslation());
      Logger.recordOutput("AprilTag/Target" + i + "/RotationDeg", data.getPose2d().getRotation().getDegrees());
    }
  }
}
