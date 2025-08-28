package frc.robot.constants;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import pwrup.frc.core.online.raspberrypi.PiNetwork;
import pwrup.frc.core.online.raspberrypi.RaspberryPi;
import lombok.AllArgsConstructor;

public class PiConstants {
  public static File configFilePath = new File(
      Filesystem.getDeployDirectory().getAbsolutePath() + "/config");

  public static class AutobahnConfig {
    public static String poseSubscribeTopic = "pos-extrapolator/robot-position";
    public static String piTechnicalLogTopic = "pi-technical-log";
    public static String odometryPublishTopic = "robot/odometry";
  }

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

  public static final PiNetwork<ProcessType> network;
  static {
    network = new PiNetwork<ProcessType>();

    network.add(new RaspberryPi<ProcessType>(
        "10.47.65.12", ProcessType.POSE_EXTRAPOLATOR));

    /*
     * network.add(new RaspberryPi<ProcessType>(
     * "10.47.65.7", ProcessType.POSE_EXTRAPOLATOR));
     */
  }
}
