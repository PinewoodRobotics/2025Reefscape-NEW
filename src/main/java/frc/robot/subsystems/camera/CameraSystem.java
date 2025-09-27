package frc.robot.subsystems.camera;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * @description represents a class of a camera position inside the robot. It has
 *              relevant util functions too.
 */
public class CameraSystem {
  // Transform2d == glorified Pose2d
  public final Transform2d robotToCamera;
  public final String cameraId;

  /**
   * 
   * @param cameraId      the name of the camera defined by "name" in camera
   *                      config in ts
   * @param cameraInRobot the transformation of the camera inside the robot (where
   *                      is it in the robot?)
   */
  public CameraSystem(String cameraId, Transform2d cameraInRobot) {
    this.robotToCamera = cameraInRobot;
    this.cameraId = cameraId;
  }

  public Transform2d getCameraToRobot() {
    return robotToCamera;
  }

  /**
   * 
   * @return Pose2d representation of the Transform2d (what is the difference...?)
   */
  public Pose2d getCameraPose() {
    return new Pose2d(robotToCamera.getTranslation(), robotToCamera.getRotation());
  }

  /**
   * 
   * @param cameraMeasurement camera-relative position of a measurement
   * @return robot relative transformation of the input camera relative
   *         measurement (before -> "where is the measurement relative to the
   *         camera", now -> "where is the measurement relative to the robot
   *         center?")
   */
  public Transform2d toRobotCentric(Transform2d cameraMeasurement) {
    return robotToCamera.plus(cameraMeasurement);
  }
}
