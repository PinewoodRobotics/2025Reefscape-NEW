package frc.robot.subsystems.camera;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.PiConstants;
import frc.robot.util.TimedRobotCentricAprilTagData;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.GeneralSensorDataOuterClass.SensorName;

public class AprilTagSubsystem extends SubsystemBase {

  /**
   * Subsystem that ingests AprilTag detections from the Pi over Autobahn,
   * keeps a short-lived cache of robot-centric tag observations keyed by tag id,
   * optionally smooths them using a small time window, and exposes query helpers
   * for commands. On each periodic, stale detections are evicted and telemetry
   * is published via AdvantageKit {@link Logger}.
   */
  private static AprilTagSubsystem self;

  /**
   * The set of camera configurations currently enabled (from
   * {@link PiConstants}).
   * Used to attach camera metadata by {@code sensorId} to inbound detections.
   */
  private final CameraSystem[] m_cameraSystems;

  /**
   * Maximum age before a tag observation is considered stale and removed.
   */
  private static final double TAG_STALE_SECONDS = 0.200;

  /**
   * If true, observations are averaged within a sliding window per tag id.
   */
  private static final boolean AVERAGE_VALUES = true;
  /**
   * Window length, in milliseconds, used when {@link #AVERAGE_VALUES} is enabled.
   */
  private static final long AVG_VALUES_EVERY_MS = 50;

  /**
   * Latest observation per tag id. Entries are evicted when they go stale.
   */
  private final Map<Integer, TimedRobotCentricAprilTagData> m_trackedTags = new HashMap<>();
  /**
   * Sliding-window accumulators keyed by tag id, used for per-tag smoothing.
   */
  private final Map<Integer, WindowAccumulator> m_windowByTag = new HashMap<>();

  /**
   * Maintains a short, time-bounded list of recent samples and returns their
   * average. A separate instance is kept per tag id.
   */
  private static class WindowAccumulator {
    /** The FPGA timestamp (seconds) at which the current window started. */
    double windowStartFPGATimeSeconds;
    final List<TimedRobotCentricAprilTagData> samples = new ArrayList<>();

    /**
     * Adds a sample. If the current window has not expired, the sample is
     * appended; otherwise the window is reset to just this sample. Returns the
     * average of all samples currently in the window.
     *
     * @param nowSeconds current FPGA timestamp in seconds
     * @param windowMs   window duration in milliseconds
     * @param sample     the new observation to add
     * @return averaged observation across the current window
     */
    TimedRobotCentricAprilTagData addSampleAndGetAverage(double nowSeconds, long windowMs,
        TimedRobotCentricAprilTagData sample) {
      if (samples.isEmpty()) {
        windowStartFPGATimeSeconds = nowSeconds;
        samples.add(sample);
        return sample;
      }

      double elapsedMs = (nowSeconds - windowStartFPGATimeSeconds) * 1000.0;
      if (elapsedMs <= windowMs) {
        samples.add(sample);
      } else {
        samples.clear();
        windowStartFPGATimeSeconds = nowSeconds;
        samples.add(sample);
      }

      return TimedRobotCentricAprilTagData
          .average(samples.toArray(new TimedRobotCentricAprilTagData[0]));
    }
  }

  public static AprilTagSubsystem GetInstance() {
    // Simple singleton guard; ensures one subsystem instance.
    if (self == null) {
      self = new AprilTagSubsystem();
    }
    return self;
  }

  public static List<TimedRobotCentricAprilTagData> GetAllTags() {
    // Convenience static wrapper for callers that do not hold the instance.
    return self.getDetectedAprilTags();
  }

  public static TimedRobotCentricAprilTagData GetBestTag() {
    return self.getBestAprilTag();
  }

  public static TimedRobotCentricAprilTagData GetTagById(int id) {
    return self.getAprilTagById(id);
  }

  private AprilTagSubsystem() {
    // Capture references to all cameras configured for this robot build.
    m_cameraSystems = PiConstants.camerasInUse;
    // Subscribe to the Pi's AprilTag stream; payloads are protobuf-encoded.
    Robot.getAutobahnClient().subscribe(PiConstants.AutobahnConfig.cameraTagsViewTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  /**
   * Autobahn subscription callback. Parses the sensor packet, filters for
   * APRIL_TAGS messages, converts each world-space tag to our
   * {@link TimedRobotCentricAprilTagData} (robot-centric pose) and updates the
   * latest-observation map, optionally applying per-tag windowed averaging.
   * Processing time is exported for telemetry.
   */
  public void subscription(byte[] payload) {
    GeneralSensorData data;
    try {
      data = GeneralSensorData.parseFrom(payload);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
      return;
    }

    if (data.getSensorName() != SensorName.APRIL_TAGS) {
      // Ignore unrelated sensor packets that arrive on the same topic.
      return;
    }

    var tags = data.getApriltags();
    double now = Timer.getFPGATimestamp();
    tags.getWorldTags().getTagsList().forEach(tag -> {
      // Build a time-stamped, robot-centric sample and attach camera metadata.
      var sample = new TimedRobotCentricAprilTagData(tag, data.getTimestamp(), now,
          getWithName(data.getSensorId()));

      if (!AVERAGE_VALUES) {
        // If smoothing is disabled, track only the most recent sample.
        m_trackedTags.put(tag.getId(), sample);
        return;
      }

      // Smooth values within a short rolling window for this tag id.
      var acc = m_windowByTag.computeIfAbsent(tag.getId(), k -> new WindowAccumulator());
      var averaged = acc.addSampleAndGetAverage(now, AVG_VALUES_EVERY_MS, sample);
      m_trackedTags.put(tag.getId(), averaged);
    });

    Logger.recordOutput("AprilTag/" + data.getSensorId() + "/ProcessTime", data.getProcessingTimeMs());
  }

  /**
   * Finds the {@link CameraSystem} definition by camera id (sensorId in packet).
   *
   * @param name camera id string from the inbound message
   * @return matching {@link CameraSystem}, or null if none registered
   */
  private CameraSystem getWithName(String name) {
    for (var i : m_cameraSystems) {
      if (i.cameraId.equals(name)) {
        return i;
      }
    }

    return null;
  }

  /**
   * Returns a snapshot list of all currently tracked tag observations.
   */
  public List<TimedRobotCentricAprilTagData> getDetectedAprilTags() {
    return new ArrayList<>(m_trackedTags.values());
  }

  /**
   * Returns the nearest tag to the robot in the XY plane, or null if none.
   */
  public TimedRobotCentricAprilTagData getBestAprilTag() {
    return getDetectedAprilTags().stream()
        .min(Comparator.comparingDouble(a -> a.getPose2d().getTranslation().getNorm()))
        .orElse(null);
  }

  /**
   * Returns the latest observation for a specific tag id, if present.
   */
  public TimedRobotCentricAprilTagData getAprilTagById(int id) {
    return getDetectedAprilTags().stream()
        .filter(tag -> tag.getId() == id)
        .findFirst()
        .orElse(null);
  }

  /**
   * Number of distinct tag ids currently tracked (not yet stale).
   */
  public int getDetectedTagCount() {
    return m_trackedTags.size();
  }

  /**
   * True if at least one non-stale tag observation exists.
   */
  public boolean hasTargets() {
    return !m_trackedTags.isEmpty();
  }

  /**
   * Alias of {@link #getBestAprilTag()} for readability in some call sites.
   */
  public TimedRobotCentricAprilTagData getClosestTag() {
    return getDetectedAprilTags().stream()
        .min(Comparator.comparingDouble(a -> a.getPose2d().getTranslation().getNorm()))
        .orElse(null);
  }

  @Override
  public void periodic() {
    // Drop stale observations based on FPGA time so consumers see fresh data.
    double now = Timer.getFPGATimestamp();
    m_trackedTags.entrySet().removeIf(e -> !e.getValue().isFresh(now, TAG_STALE_SECONDS));

    var current = new ArrayList<>(m_trackedTags.values());

    // Publish high-level telemetry and per-target details for debugging.
    Logger.recordOutput("AprilTag/DetectedTargets", current.size());
    Logger.recordOutput("AprilTag/HasTargets", !current.isEmpty());
    Logger.recordOutput("AprilTag/AveragingEnabled", AVERAGE_VALUES);

    for (int i = 0; i < current.size(); i++) {
      var data = current.get(i);
      Logger.recordOutput("AprilTag/Target" + i + "/ID", data.getId());
      Logger.recordOutput("AprilTag/Target" + i + "/Pose2d", data.getPose2d());
      Logger.recordOutput("AprilTag/Target" + i + "/Translation2d", data.getPose2d().getTranslation());
      Logger.recordOutput("AprilTag/Target" + i + "/RotationDeg", data.getPose2d().getRotation().getDegrees());
    }
  }
}
