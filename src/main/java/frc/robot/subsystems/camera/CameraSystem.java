package frc.robot.subsystems.camera;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class CameraSystem extends PhotonCamera {
    public final Transform2d robotToCamera;

    public CameraSystem(String cameraName, Transform2d cameraInRobot) {
        super(cameraName);
        this.robotToCamera = cameraInRobot;
    }

    public Transform2d getCameraToRobot() {
        return robotToCamera;
    }

    public Pose2d getCameraPose() {
        return new Pose2d(robotToCamera.getTranslation(), robotToCamera.getRotation());
    }

    public Transform2d toRobotCentric(Transform2d cameraMeasurement) {
        return robotToCamera.plus(cameraMeasurement);
    }
}
