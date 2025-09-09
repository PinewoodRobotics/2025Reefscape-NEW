package frc.robot.util;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import lombok.Getter;

import org.photonvision.targeting.PhotonTrackedTarget;

@Getter
public class TimedAprilTagData {
    private final int id;
    private final SimpleMatrix T_tagInRobot;
    private final Pose2d pose2d;
    private final double resultTimestampSeconds;
    private final double lastSeenFPGATimeSeconds;

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
        Transform3d cameraToTarget = target.getBestCameraToTarget();

        Pose2d P_tagInCamera = new Pose2d(cameraToTarget.getTranslation().toTranslation2d(),
                cameraToTarget.getRotation().toRotation2d());

        SimpleMatrix T_tagInCamera = CustomMath.fromPose2dToMatrix(P_tagInCamera);
        SimpleMatrix T_tagInRobot = CustomMath.toRobotRelative(T_tagInCamera, T_cameraInRobot);

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
}
