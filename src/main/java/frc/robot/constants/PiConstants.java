package frc.robot.constants;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Filesystem;
import lombok.AllArgsConstructor;
import pwrup.frc.core.online.raspberrypi.AutomaticPiNetwork;
import pwrup.frc.core.online.raspberrypi.ConstrainedProcess;
import pwrup.frc.core.online.raspberrypi.WeightedProcess;

public class PiConstants {
  @AllArgsConstructor
  public static enum ProcessType implements WeightedProcess {
    POSE_EXTRAPOLATOR("position-extrapolator"),
    PATHFINDING("pathfinding"),
    APRIL_TAG_DETECTOR("april-server"),
    MINECRAFT_SERVER("minecraft-server");

    private final String name;

    @Override
    public String toString() {
      return name;
    }

    @Override
    public double getWeight() {
      switch (this) {
        case POSE_EXTRAPOLATOR:
          return 0.5;
        case APRIL_TAG_DETECTOR:
          return 1.0;
        case PATHFINDING:
          return 1;
        case MINECRAFT_SERVER:
          return 1;
      }

      return 0.0;
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
  public static final AutomaticPiNetwork<ProcessType> network = new AutomaticPiNetwork<ProcessType>(4,
      ProcessType.APRIL_TAG_DETECTOR, /* ProcessType.APRIL_TAG_DETECTOR, */ ProcessType.POSE_EXTRAPOLATOR);

  static {
    AutomaticPiNetwork.AddConstrainedProcesses(
        new ConstrainedProcess<>(ProcessType.APRIL_TAG_DETECTOR, "tripli"));
    /*
     * AutomaticPiNetwork.AddConstrainedProcesses(
     * new ConstrainedProcess<>(ProcessType.APRIL_TAG_DETECTOR, "agathaking"));
     */
  }
}
