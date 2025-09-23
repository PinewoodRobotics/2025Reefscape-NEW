package frc.robot.util;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.camera.CameraSystem;
import edu.wpi.first.math.geometry.Transform2d;
import lombok.Getter;

import org.photonvision.targeting.PhotonTrackedTarget;

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

  private TimedRobotCentricAprilTagData(int id, double frameTimestampSeconds,
      double lastSeenFPGATimeSeconds, Transform2d cameraToTag, CameraSystem cameraSystem) {
    this.id = id;
    this.resultTimestampSeconds = frameTimestampSeconds;
    this.lastSeenFPGATimeSeconds = lastSeenFPGATimeSeconds;
    this.cameraToTag = cameraToTag;
    this.robotToTag = cameraSystem.toRobotCentric(cameraToTag);
    this.pose2d = new Pose2d(robotToTag.getTranslation(), robotToTag.getRotation());
  }

  private TimedRobotCentricAprilTagData(int id, double frameTimestampSeconds,
      double lastSeenFPGATimeSeconds, Pose2d pose2d) {
    this.id = id;
    this.resultTimestampSeconds = frameTimestampSeconds;
    this.lastSeenFPGATimeSeconds = lastSeenFPGATimeSeconds;
    this.pose2d = pose2d;
  }

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

  public boolean isFresh(double nowFPGASeconds, double staleSeconds) {
    return (nowFPGASeconds - this.lastSeenFPGATimeSeconds) <= staleSeconds;
  }

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
