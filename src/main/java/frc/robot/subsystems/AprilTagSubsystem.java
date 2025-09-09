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

public class AprilTagSubsystem extends SubsystemBase {

  private static AprilTagSubsystem self;

  private final PiConstants.CameraConstants.CameraMount[] m_cameraMounts;

  private static final double TAG_STALE_SECONDS = 0.200;

  private final Map<Integer, AprilTagData> m_trackedTags = new HashMap<>();

  public static class AprilTagData {
    public final int id;
    public final SimpleMatrix T_tagInRobot;
    public final Pose2d pose2d;
    public final double resultTimestampSeconds;
    public final double lastSeenFPGATimeSeconds;

    private AprilTagData(int id, SimpleMatrix T_tagInRobot, double frameTimestampSeconds,
        double lastSeenFPGATimeSeconds) {
      this.id = id;
      this.resultTimestampSeconds = frameTimestampSeconds;
      this.lastSeenFPGATimeSeconds = lastSeenFPGATimeSeconds;
      this.T_tagInRobot = T_tagInRobot;
      this.pose2d = CustomMath.fromTransformationMatrix2dToPose2d(T_tagInRobot);
    }

    public static AprilTagData fromDetection(PhotonTrackedTarget target, SimpleMatrix T_cameraInRobot,
        double frameTimestampSeconds, double nowFPGASeconds) {
      Transform3d cameraToTarget = target.getBestCameraToTarget();

      Pose2d P_tagInCamera = new Pose2d(cameraToTarget.getTranslation().toTranslation2d(),
          cameraToTarget.getRotation().toRotation2d());

      SimpleMatrix T_tagInCamera = CustomMath.fromPose2dToMatrix(P_tagInCamera);
      SimpleMatrix T_tagInRobot = CustomMath.toRobotRelative(T_tagInCamera, T_cameraInRobot);

      return new AprilTagData(target.getFiducialId(), T_tagInRobot, frameTimestampSeconds, nowFPGASeconds);
    }

    public AprilTagData updateFrom(PhotonTrackedTarget target, SimpleMatrix T_cameraInRobot,
        double frameTimestampSeconds, double nowFPGASeconds) {
      if (frameTimestampSeconds > this.resultTimestampSeconds) {
        return AprilTagData.fromDetection(target, T_cameraInRobot, frameTimestampSeconds, nowFPGASeconds);
      }

      return new AprilTagData(this.id, this.T_tagInRobot, this.resultTimestampSeconds, nowFPGASeconds);
    }

    public boolean isFresh(double nowFPGASeconds, double staleSeconds) {
      return (nowFPGASeconds - this.lastSeenFPGATimeSeconds) <= staleSeconds;
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

  public static AprilTagData GetBestTag() {
    return self.getBestAprilTag();
  }

  public static AprilTagData GetTagById(int id) {
    return self.getAprilTagById(id);
  }

  private AprilTagSubsystem() {
    m_cameraMounts = PiConstants.CameraConstants.photonCamerasInUse;
  }

  public List<AprilTagData> getDetectedAprilTags() {
    return new ArrayList<>(m_trackedTags.values());
  }

  public AprilTagData getBestAprilTag() {
    return getDetectedAprilTags().stream()
        .min(Comparator.comparingDouble(a -> a.pose2d.getTranslation().getNorm()))
        .orElse(null);
  }

  public AprilTagData getAprilTagById(int id) {
    return getDetectedAprilTags().stream()
        .filter(tag -> tag.id == id)
        .findFirst()
        .orElse(null);
  }

  public int getDetectedTagCount() {
    return m_trackedTags.size();
  }

  public boolean hasTargets() {
    return !m_trackedTags.isEmpty();
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
        AprilTagData prev = m_trackedTags.get(id);
        if (prev == null) {
          m_trackedTags.put(id, AprilTagData.fromDetection(target, mount.T_cameraInRobot, frameTimestamp, nowFPGA));
        } else {
          m_trackedTags.put(id, prev.updateFrom(target, mount.T_cameraInRobot, frameTimestamp, nowFPGA));
        }
      }
    }

    double now = Timer.getFPGATimestamp();
    m_trackedTags.entrySet().removeIf(e -> !e.getValue().isFresh(now, TAG_STALE_SECONDS));

    List<AprilTagData> current = new ArrayList<>(m_trackedTags.values());

    Logger.recordOutput("AprilTag/DetectedTargets", current.size());
    Logger.recordOutput("AprilTag/HasTargets", !current.isEmpty());
    Logger.recordOutput("AprilTag/LatestResultsCount", latestResults.size());

    for (int i = 0; i < current.size(); i++) {
      AprilTagData data = current.get(i);
      Logger.recordOutput("AprilTag/Target" + i + "/ID", data.id);
      Logger.recordOutput("AprilTag/Target" + i + "/Pose2d", data.pose2d);
      Logger.recordOutput("AprilTag/Target" + i + "/Translation2d", data.pose2d.getTranslation());
      Logger.recordOutput("AprilTag/Target" + i + "/RotationDeg", data.pose2d.getRotation().getDegrees());
    }
  }
}
