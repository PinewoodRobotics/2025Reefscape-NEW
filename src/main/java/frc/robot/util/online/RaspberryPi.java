package frc.robot.util.online;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import com.google.protobuf.InvalidProtocolBufferException;

import frc.robot.util.Communicator;
import proto.WatchDogMessage.MessageRetrievalConfirmation;
import proto.WatchDogMessage.ProcessType;
import proto.WatchDogMessage.StartupMessage;

public class RaspberryPi {

  public final Address address;
  public final ProcessType[] processesToRun;
  public final String name;
  private boolean shouldReSend = false;

  public RaspberryPi(Address addr, ProcessType[] processes, String name) {
    this.address = addr;
    this.processesToRun = processes;
    this.name = name;
  }

  public void initialize(File configFile) {
    Communicator.subscribeAutobahn(name + "/watchdog/message_retrieval_confirmation", this::onMessage);

    this.shouldReSend = true;
    new Thread(() -> {
      while (this.shouldReSend) {
        Communicator.sendMessageToSpecificAutobahn(address, "config",
            RaspberryPi.constructConfigMessage(configFile, processesToRun));

        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    });
  }

  private void onMessage(byte[] bytes) {
    try {
      MessageRetrievalConfirmation message = MessageRetrievalConfirmation.parseFrom(bytes);
      if (message.getReceived()) {
        this.shouldReSend = false;
      }
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
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
