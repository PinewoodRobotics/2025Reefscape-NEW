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
        .min(Comparator.comparingDouble(a -> a.getRobotToTag().getTranslation().getNorm()))
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
        .min(Comparator.comparingDouble(a -> a.getRobotToTag().getTranslation().getNorm()))
        .orElse(null);
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> latestResults = new ArrayList<>();

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
        m_trackedTags.put(id,
            new TimedRobotCentricAprilTagData(target, frameTimestamp, nowFPGA, mount));
      }
    }

    double now = Timer.getFPGATimestamp();
    m_trackedTags.entrySet().removeIf(e -> !e.getValue().isFresh(now, TAG_STALE_SECONDS));

    List<TimedRobotCentricAprilTagData> current = new ArrayList<>(m_trackedTags.values());

    Logger.recordOutput("AprilTag/DetectedTargets", current.size());
    Logger.recordOutput("AprilTag/HasTargets", !current.isEmpty());
    Logger.recordOutput("AprilTag/LatestResultsCount", latestResults.size());

    for (int i = 0; i < current.size(); i++) {
      TimedRobotCentricAprilTagData data = current.get(i);
      Logger.recordOutput("AprilTag/Target" + i + "/ID", data.getId());
      Logger.recordOutput("AprilTag/Target" + i + "/Pose2d", data.getPose2d());
      Logger.recordOutput("AprilTag/Target" + i + "/Translation2d", data.getPose2d().getTranslation());
      Logger.recordOutput("AprilTag/Target" + i + "/RotationDeg", data.getPose2d().getRotation().getDegrees());
    }
  }
}
