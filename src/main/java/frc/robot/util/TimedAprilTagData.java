package frc.robot.util;

import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Transform2d;
import lombok.Getter;

import org.photonvision.targeting.PhotonTrackedTarget;

@Getter
public class TimedAprilTagData implements LoggableInputs {
    private int id;
    private SimpleMatrix T_tagInRobot;
    private Pose2d pose2d;
    private double resultTimestampSeconds;
    private double lastSeenFPGATimeSeconds;

    private TimedAprilTagData(int id, SimpleMatrix T_tagInRobot, double frameTimestampSeconds,
            double lastSeenFPGATimeSeconds) {
        this.id = id;
        this.resultTimestampSeconds = frameTimestampSeconds;
        this.lastSeenFPGATimeSeconds = lastSeenFPGATimeSeconds;
        this.T_tagInRobot = T_tagInRobot;
        this.pose2d = CustomMath.fromTransformationMatrix2dToPose2d(T_tagInRobot);
    }

    public static TimedAprilTagData fromDetection(PhotonTrackedTarget target, SimpleMatrix T_cameraInRobot,
            double frameTimestampSeconds, double nowFPGASeconds) {
        // PhotonVision provides the transform from camera to target (tag)
        Transform3d cameraToTarget = target.getBestCameraToTarget();

        // Convert to 2D for planar composition
        Pose2d tagInCamera = new Pose2d(
                cameraToTarget.getTranslation().toTranslation2d(),
                cameraToTarget.getRotation().toRotation2d());

        // Recover camera pose in robot coordinates from the provided transform matrix
        Pose2d cameraInRobot = CustomMath.fromTransformationMatrix2dToPose2d(T_cameraInRobot);

        // Compose: robot->tag = robot->camera o camera->tag
        Pose2d tagInRobot = cameraInRobot.transformBy(new Transform2d(tagInCamera.getTranslation(),
                tagInCamera.getRotation()));

        SimpleMatrix T_tagInRobot = CustomMath.fromPose2dToMatrix(tagInRobot);

        return new TimedAprilTagData(target.getFiducialId(), T_tagInRobot, frameTimestampSeconds, nowFPGASeconds);
    }

    public TimedAprilTagData updateFrom(PhotonTrackedTarget target, SimpleMatrix T_cameraInRobot,
            double frameTimestampSeconds, double nowFPGASeconds) {
        if (frameTimestampSeconds > this.resultTimestampSeconds) {
            return TimedAprilTagData.fromDetection(target, T_cameraInRobot, frameTimestampSeconds, nowFPGASeconds);
        }

        return new TimedAprilTagData(this.id, this.T_tagInRobot, this.resultTimestampSeconds, nowFPGASeconds);
    }

    public boolean isFresh(double nowFPGASeconds, double staleSeconds) {
        return (nowFPGASeconds - this.lastSeenFPGATimeSeconds) <= staleSeconds;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("id", id);
        table.put("T_tagInRobot", T_tagInRobot.toString());
        table.put("pose2d", pose2d);
        table.put("resultTimestampSeconds", resultTimestampSeconds);
        table.put("lastSeenFPGATimeSeconds", lastSeenFPGATimeSeconds);
    }

    @Override
    public void fromLog(LogTable table) {
        id = table.get("id", id);
        pose2d = table.get("pose2d", pose2d);
        resultTimestampSeconds = table.get("resultTimestampSeconds", resultTimestampSeconds);
        lastSeenFPGATimeSeconds = table.get("lastSeenFPGATimeSeconds", lastSeenFPGATimeSeconds);
    }
}
