package frc.robot.util;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.camera.CameraSystem;
import lombok.Getter;
import proto.sensor.Apriltags.ProcessedTag;

/**
 * @description represents a class of a timed measurement of an AprilTag. It
 *              also implements LoggableInputs so that it can be logged.
 */
public class TimedRobotCentricAprilTagData implements LoggableInputs {
  @Getter
  private Transform2d cameraToTag;
  private Transform2d robotToTag;
  private double resultTimestampSeconds;
  private double lastSeenFPGATimeSeconds;

  @Getter
  private Pose2d pose2d;
  @Getter
  private int id;

  /**
   * 
   * @param id                      the id of the AprilTag
   * @param frameTimestampSeconds   the time the AprilTag was detected
   * @param lastSeenFPGATimeSeconds the time the AprilTag was last seen
   * @param cameraToTag             the camera-relative position of the AprilTag
   * @param cameraSystem            the camera system (used to convert the
   *                                camera-relative position to a robot-relative
   *                                position)
   */
  private TimedRobotCentricAprilTagData(int id, double frameTimestampSeconds,
      double lastSeenFPGATimeSeconds, Transform2d cameraToTag, CameraSystem cameraSystem) {
    this.id = id;
    this.resultTimestampSeconds = frameTimestampSeconds;
    this.lastSeenFPGATimeSeconds = lastSeenFPGATimeSeconds;
    this.cameraToTag = cameraToTag;
    this.robotToTag = cameraSystem.toRobotCentric(cameraToTag);
    this.pose2d = new Pose2d(robotToTag.getTranslation(), robotToTag.getRotation());
  }

  /**
   * 
   * @param id                      the id of the AprilTag
   * @param frameTimestampSeconds   the time the AprilTag was detected
   * @param lastSeenFPGATimeSeconds the time the AprilTag was last seen
   * @param pose2d                  the robot-relative position of the AprilTag
   * @used by average() to clone a tag
   */
  private TimedRobotCentricAprilTagData(int id, double frameTimestampSeconds,
      double lastSeenFPGATimeSeconds, Pose2d pose2d) {
    this.id = id;
    this.resultTimestampSeconds = frameTimestampSeconds;
    this.lastSeenFPGATimeSeconds = lastSeenFPGATimeSeconds;
    this.pose2d = pose2d;
  }

  /**
   * 
   * @param target                the PhotonTrackedTarget
   * @param frameTimestampSeconds the time the AprilTag was detected
   * @param nowFPGASeconds        the current time
   * @param cameraSystem          the camera system (used to convert the
   *                              camera-relative position to a robot-relative
   *                              position)
   */
  public TimedRobotCentricAprilTagData(PhotonTrackedTarget target,
      double frameTimestampSeconds, double nowFPGASeconds, CameraSystem cameraSystem) {
    this(
        target.getFiducialId(),
        frameTimestampSeconds,
        nowFPGASeconds,
        new Transform2d(
            target.getBestCameraToTarget().getX(),
            target.getBestCameraToTarget().getY(),
            target.getBestCameraToTarget().getRotation().toRotation2d()),
        cameraSystem);
  }

  /**
   * 
   * @param target                the ProcessedTag
   * @param frameTimestampSeconds the time the AprilTag was detected
   * @param nowFPGASeconds        the current time
   * @param cameraSystem          the camera system (used to convert the
   *                              camera-relative position to a robot-relative
   *                              position)
   */
  public TimedRobotCentricAprilTagData(ProcessedTag target,
      double frameTimestampSeconds, double nowFPGASeconds, CameraSystem cameraSystem) {
    this(
        target.getId(),
        frameTimestampSeconds,
        nowFPGASeconds,
        new Transform2d(
            new Translation2d(target.getPositionWPILib().getX(), target.getPositionWPILib().getY()),
            new Rotation2d(target.getRotationWPILib().getYaw())),
        cameraSystem);
  }

  /**
   * 
   * @param nowFPGASeconds the current time
   * @param staleSeconds   the stale time
   * @return true if the AprilTag is fresh
   */
  public boolean isFresh(double nowFPGASeconds, double staleSeconds) {
    return (nowFPGASeconds - this.lastSeenFPGATimeSeconds) <= staleSeconds;
  }

  /**
   * 
   * @param tags the tags to average
   * @return the averaged tag
   */
  public static TimedRobotCentricAprilTagData average(TimedRobotCentricAprilTagData... tags) {
    double totalX = 0;
    double totalY = 0;
    double totalCos = 0;
    double totalSin = 0;

    for (TimedRobotCentricAprilTagData tag : tags) {
      totalX += tag.getPose2d().getX();
      totalY += tag.getPose2d().getY();
      totalCos += tag.getPose2d().getRotation().getCos();
      totalSin += tag.getPose2d().getRotation().getSin();
    }

    double avgX = totalX / tags.length;
    double avgY = totalY / tags.length;
    double avgCos = totalCos / tags.length;
    double avgSin = totalSin / tags.length;

    return new TimedRobotCentricAprilTagData(tags[0].getId(), tags[0].resultTimestampSeconds,
        tags[0].lastSeenFPGATimeSeconds, new Pose2d(avgX, avgY, new Rotation2d(avgCos, avgSin)));
  }

  @Override
  public void toLog(LogTable table) {
    table.put("id", id);
    table.put("resultTimestampSeconds", resultTimestampSeconds);
    table.put("lastSeenFPGATimeSeconds", lastSeenFPGATimeSeconds);
    table.put("pose2d", pose2d);
  }

  @Override
  public void fromLog(LogTable table) {
    id = table.get("id", id);
    resultTimestampSeconds = table.get("resultTimestampSeconds", resultTimestampSeconds);
    lastSeenFPGATimeSeconds = table.get("lastSeenFPGATimeSeconds", lastSeenFPGATimeSeconds);
  }
}
