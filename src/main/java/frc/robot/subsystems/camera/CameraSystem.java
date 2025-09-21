package frc.robot.subsystems.camera;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class CameraSystem {
  public final Transform2d robotToCamera;
  public final String cameraId;

  public CameraSystem(String cameraId, Transform2d cameraInRobot) {
    this.robotToCamera = cameraInRobot;
    this.cameraId = cameraId;
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
