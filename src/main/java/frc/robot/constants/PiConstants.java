package frc.robot.constants;

import java.io.File;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.camera.CameraSystem;
import frc.robot.util.CustomMath;
import lombok.AllArgsConstructor;
import pwrup.frc.core.online.raspberrypi.PiNetwork;
import pwrup.frc.core.online.raspberrypi.RaspberryPi;

public class PiConstants {
  @AllArgsConstructor
  public static enum ProcessType {
    POSE_EXTRAPOLATOR("position-extrapolator"),
    APRIL_TAG_DETECTOR("april-server");

    private final String name;

    @Override
    public String toString() {
      return name;
    }
  }

  public static class AutobahnConfig {
    public static String poseSubscribeTopic = "pos-extrapolator/robot-position";
    public static String piTechnicalLogTopic = "pi-technical-log";
    public static String odometryPublishTopic = "robot/odometry";
    public static String cameraViewTopic = "apriltag/camera";
    public static String cameraTagsViewTopic = "apriltag/tags";
  }

  public static File configFilePath = new File(
      Filesystem.getDeployDirectory().getAbsolutePath() + "/config");

  public static final PiNetwork<ProcessType> network;
  static {
    network = new PiNetwork<ProcessType>();
  }

  public static final CameraSystem[] camerasInUse = new CameraSystem[] {
      new CameraSystem(
          "front_left", // left camera facing 45 deg inwards
          new Transform2d(0.33, 0.33, new Rotation2d(Math.toRadians(-45)))),
      new CameraSystem(
          "front_right", // right camera facing 45 deg inwards
          new Transform2d(0.33, -0.33, new Rotation2d(Math.toRadians(45)))),
  };
}
