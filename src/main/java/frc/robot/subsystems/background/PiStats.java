package frc.robot.subsystems.background;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import autobahn.client.NamedCallback;
import frc.robot.Robot;
import frc.robot.constants.PiConstants;
import proto.status.StateLoggingOuterClass.StateLogging;
import proto.status.PiStatusOuterClass.LogMessage;
import proto.status.PiStatusOuterClass.PiStatus;
import proto.status.PiStatusOuterClass.Ping;
import proto.status.PiStatusOuterClass.Pong;
import proto.status.StateLoggingOuterClass.DataEntry;
import proto.status.StateLoggingOuterClass.DataType;
import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PiStats extends SubsystemBase {

  private static PiStats self;
  private final String piAkitTopic = PiConstants.AutobahnConfig.piTechnicalLogTopic + "/akit";
  private final String piStatusTopic = PiConstants.AutobahnConfig.piTechnicalLogTopic + "/status";
  private final long pingTimeIntervalMs = 200;
  private long lastPingTimeMs = 0;

  public static PiStats GetInstance() {
    if (self == null) {
      self = new PiStats();
    }

    return self;
  }

  public PiStats() {
    super();

    Robot.communication.subscribe(piAkitTopic,
        NamedCallback.FromConsumer(this::subscriptionAkit));

    Robot.communication.subscribe(piStatusTopic,
        NamedCallback.FromConsumer(this::subscriptionStats));

    Robot.communication.subscribe(PiConstants.AutobahnConfig.piTechnicalLogTopic,
        NamedCallback.FromConsumer(this::logSubscription));

    Robot.communication.subscribe("pi-pong",
        NamedCallback.FromConsumer(this::subscriptionPong));
  }

  private void logSubscription(byte[] data) {
    try {
      LogMessage piLog = LogMessage.parseFrom(data);
      String name = piLog.getPiName() == null ? "unknown-pi" : piLog.getPiName();
      Logger.recordOutput(name + "/piLog/message", piLog.getMessage());
      Logger.recordOutput(name + "/piLog/type", piLog.getType().toString());
      Logger.recordOutput(name + "/piLog/prefix", piLog.getPrefix());
      Logger.recordOutput(name + "/piLog/color", piLog.getColor());

      System.out.println(piLog.getMessage());
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  private void subscriptionStats(byte[] data) {
    try {
      PiStatus piStatus = PiStatus.parseFrom(data);
      Logger.recordOutput(piStatus.getPiName() + "/piStats/cpuUsage", piStatus.getCpuUsageTotal());
      Logger.recordOutput(piStatus.getPiName() + "/piStats/memoryUsage", piStatus.getMemoryUsage());
      Logger.recordOutput(piStatus.getPiName() + "/piStats/diskUsage", piStatus.getDiskUsage());
      Logger.recordOutput(piStatus.getPiName() + "/piStats/networkUsageIn", piStatus.getNetUsageIn());
      Logger.recordOutput(piStatus.getPiName() + "/piStats/networkUsageOut", piStatus.getNetUsageOut());
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  private void subscriptionAkit(byte[] data) {
    try {
      StateLogging stateLogging = StateLogging.parseFrom(data);
      for (DataEntry entry : stateLogging.getEntriesList()) {
        String name = stateLogging.getName();
        DataType type = entry.getType();
        String entryName = name + stateLogging.getTopic();

        switch (type) {
          case BOOL:
            mskrBool(entryName, entry.getBoolValuesList());
            break;
          case INT:
            mskrInt(entryName, entry.getIntValuesList());
            break;
          case FLOAT:
            mskrFloat(entryName, entry.getFloatValuesList());
            break;
          case STRING:
            mskrString(entryName, entry.getStringValuesList());
            break;
          default:
            // Unknown or unsupported type, do nothing
            break;
        }
      }
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  // Very jankey solution but haven't found an alternative yet
  private void mskrBool(String name, List<Boolean> bools) {
    if (bools.size() == 1) {
      Logger.recordOutput(name, bools.get(0));
    } else if (bools.size() > 1) {
      boolean[] arr = new boolean[bools.size()];
      for (int i = 0; i < bools.size(); i++)
        arr[i] = bools.get(i) != null ? bools.get(i) : false;
      Logger.recordOutput(name, arr);
    }
  }

  private void mskrInt(String name, List<Integer> ints) {
    if (ints.size() == 1) {
      Logger.recordOutput(name, ints.get(0));
    } else if (ints.size() > 1) {
      int[] arr = new int[ints.size()];
      for (int i = 0; i < ints.size(); i++)
        arr[i] = ints.get(i);
      Logger.recordOutput(name, arr);
    }
  }

  private void mskrFloat(String name, List<Float> floats) {
    if (floats.size() == 1) {
      Logger.recordOutput(name, floats.get(0));
    } else if (floats.size() > 1) {
      double[] arr = new double[floats.size()];
      for (int i = 0; i < floats.size(); i++)
        arr[i] = floats.get(i);
      Logger.recordOutput(name, arr);
    }
  }

  private void mskrString(String name, List<String> strings) {
    if (strings.size() == 1) {
      Logger.recordOutput(name, strings.get(0));
    } else if (strings.size() > 1) {
      Logger.recordOutput(name, strings.toArray(new String[0]));
    }
  }

  @Override
  public void periodic() {
    long currentTimeMs = System.currentTimeMillis();
    if (currentTimeMs - lastPingTimeMs > pingTimeIntervalMs) {
      Robot.communication.publish("pi-ping",
          Ping.newBuilder().setTimestamp(currentTimeMs).build().toByteArray());
      lastPingTimeMs = currentTimeMs;
    }
  }

  private void subscriptionPong(byte[] data) {
    try {
      Pong pong = Pong.parseFrom(data);
      long currentTime = System.currentTimeMillis();
      long latencyToAndFrom = currentTime - pong.getTimestampMsOriginal();
      long latencyOneWay = currentTime - pong.getTimestampMsReceived();

      String piName = pong.getPiName();

      Logger.recordOutput(piName + "/ping/latencyToAndFrom", latencyToAndFrom);
      Logger.recordOutput(piName + "/ping/latencyOneWay", latencyOneWay);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }
}
