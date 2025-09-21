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

@Getter
public class TimedRobotCentricAprilTagData implements LoggableInputs {
  private int id;
  private Transform2d cameraToTag;
  private Transform2d robotToTag;
  private double resultTimestampSeconds;
  private double lastSeenFPGATimeSeconds;
  private Pose2d pose2d;

  private TimedRobotCentricAprilTagData(int id, double frameTimestampSeconds,
      double lastSeenFPGATimeSeconds, Transform2d cameraToTag, CameraSystem cameraSystem) {
    this.id = id;
    this.resultTimestampSeconds = frameTimestampSeconds;
    this.lastSeenFPGATimeSeconds = lastSeenFPGATimeSeconds;
    this.cameraToTag = cameraToTag;
    this.robotToTag = cameraSystem.toRobotCentric(cameraToTag);
    this.pose2d = new Pose2d(robotToTag.getTranslation(), robotToTag.getRotation());
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

  public boolean isFresh(double nowFPGASeconds, double staleSeconds) {
    return (nowFPGASeconds - this.lastSeenFPGATimeSeconds) <= staleSeconds;
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
