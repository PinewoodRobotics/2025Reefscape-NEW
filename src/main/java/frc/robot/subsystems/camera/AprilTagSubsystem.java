package frc.robot.subsystems.camera;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.constants.PiConstants;
import frc.robot.util.TimedRobotCentricAprilTagData;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;

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
    m_cameraSystems = PiConstants.camerasInUse;
    Robot.getAutobahnClient().subscribe(PiConstants.AutobahnConfig.cameraTagsViewTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  public void subscription(byte[] payload) {
    GeneralSensorData data;
    try {
      data = GeneralSensorData.parseFrom(payload);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
      return;
    }

    if (data.getSensorName() != SensorName.APRIL_TAGS) {
      return;
    }

    var tags = data.getApriltags();
    tags.getWorldTags().getTagsList().forEach(tag -> {
      m_trackedTags.put(tag.getId(),
          new TimedRobotCentricAprilTagData(tag, data.getTimestamp(), Timer.getFPGATimestamp(), m_cameraSystems[0]));
    });
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
    double now = Timer.getFPGATimestamp();
    m_trackedTags.entrySet().removeIf(e -> !e.getValue().isFresh(now, TAG_STALE_SECONDS));

    var current = new ArrayList<>(m_trackedTags.values());

    Logger.recordOutput("AprilTag/DetectedTargets", current.size());
    Logger.recordOutput("AprilTag/HasTargets", !current.isEmpty());
    for (int i = 0; i < current.size(); i++) {
      var data = current.get(i);
      Logger.recordOutput("AprilTag/Target" + i + "/ID", data.getId());
      Logger.recordOutput("AprilTag/Target" + i + "/Pose2d", data.getPose2d());
      Logger.recordOutput("AprilTag/Target" + i + "/Translation2d", data.getPose2d().getTranslation());
      Logger.recordOutput("AprilTag/Target" + i + "/RotationDeg", data.getPose2d().getRotation().getDegrees());
    }
  }
}
