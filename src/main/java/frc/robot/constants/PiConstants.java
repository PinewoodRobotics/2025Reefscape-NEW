package frc.robot.constants;

import java.io.File;

import autobahn.client.Address;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.camera.CameraSystem;
import lombok.AllArgsConstructor;
import pwrup.frc.core.online.raspberrypi.PiNetwork;

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
    public static String cameraTagsViewTopic = "apriltag/tag";
  }

  public static File configFilePath = new File(
      Filesystem.getDeployDirectory().getAbsolutePath() + "/config");

  // a network is a glorified list of a custom "RaspberryPi" class that implements
  // some of the util methods (startProcess/stopProcess etc.). This is a
  // declaration and you add stuff inside the static block so not to clutter
  // things up.
  public static final PiNetwork<ProcessType> network = new PiNetwork<ProcessType>();
  static {
    // pi #1.
    network.add(new Address("raspberrypi1.local", 8080), ProcessType.APRIL_TAG_DETECTOR, ProcessType.POSE_EXTRAPOLATOR); // specify
                                                                                                                         // the
                                                                                                                         // processes
                                                                                                                         // you
    // want this to run after the
    // address.
  }

  // (NOTE: CAMERAID HAS TO BE THE SAME AS IN TS CONFIG!)
  public static final CameraSystem[] camerasInUse = new CameraSystem[] {
      new CameraSystem(
          "front_left", // left camera facing 45 deg inwards
          new Transform2d(0.33, 0.33, new Rotation2d(Math.toRadians(-45)))),
      new CameraSystem(
          "front_right", // right camera facing 45 deg inwards
          new Transform2d(0.33, -0.33, new Rotation2d(Math.toRadians(45)))),
  };
}
