package frc.robot.constants;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.online.Address;
import frc.robot.util.online.RaspberryPi;
import proto.WatchDogMessage.ProcessType;

public class AutobahnConstants {

  public static final boolean kEnableOffline = false;
  public static final File configFilePath = new File(
      Filesystem.getDeployDirectory().getAbsolutePath() + "/config.json");

  public static final RaspberryPi tripoli = new RaspberryPi(
      new Address("10.47.65.7", 8080),
      new ProcessType[] {},
      "tripoli");

  public static final RaspberryPi agatha_king = new RaspberryPi(
      new Address("10.47.65.12", 8080),
      new ProcessType[] { ProcessType.CAMERA_PROCESSING },
      "agatha_king");

  public static final RaspberryPi donnager = new RaspberryPi(
      new Address("10.47.65.13", 8080),
      new ProcessType[] { ProcessType.CAMERA_PROCESSING },
      "donnager");

  public static final RaspberryPi[] all = new RaspberryPi[] { agatha_king };
}
