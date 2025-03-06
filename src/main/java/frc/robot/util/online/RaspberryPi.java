package frc.robot.util.online;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import frc.robot.util.Communicator;
import proto.WatchDogMessage.ProcessType;
import proto.WatchDogMessage.StartupMessage;

public class RaspberryPi {

  public final Address address;
  public final ProcessType[] processesToRun;

  public RaspberryPi(Address addr, ProcessType[] processes) {
    this.address = addr;
    this.processesToRun = processes;
  }

  public void initialize(File configFile) {
    Communicator.sendMessageToSpecificAutobahn(address, "config",
        RaspberryPi.constructConfigMessage(configFile, processesToRun));
  }

  public static byte[] constructConfigMessage(
      File configFileJson,
      ProcessType... processes) {
    StartupMessage.Builder message = StartupMessage.newBuilder();
    try {
      String jsonString = new String(
          Files.readAllBytes(configFileJson.toPath()));

      message.setJsonConfig(jsonString);
    } catch (IOException e) {
      e.printStackTrace();
    }

    for (ProcessType process : processes) {
      message.addPyTasks(process);
    }

    message.setAbortPrevious(true);

    return message.build().toByteArray();
  }
}
