package frc.robot.util;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.camera.CameraSystem;
import edu.wpi.first.math.geometry.Transform2d;
import lombok.Getter;

import org.photonvision.targeting.PhotonTrackedTarget;

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
