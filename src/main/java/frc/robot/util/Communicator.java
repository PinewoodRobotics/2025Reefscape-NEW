package frc.robot.util;

import java.util.function.Consumer;

import org.pwrup.util.IPublisher;

import com.google.gson.Gson;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.online.Address;
import frc.robot.util.online.Autobahn;

public class Communicator implements IPublisher {

  private final Gson gson = new Gson();
  private static Autobahn autobahn;

  @Override
  public void publish(String arg0, Object arg1, Class<?> arg2) {
    var networkTable = NetworkTableInstance.getDefault();
    networkTable.getEntry(arg0).setString(gson.toJson(arg1, arg2));
  }

  public static void init(Autobahn autobahn) {
    Communicator.autobahn = autobahn;
  }

  public static void sendMessageAutobahn(String pubTopic, byte[] message) {
    if (autobahn == null) {
      return;
    }

    autobahn.publish(pubTopic, message);
  }

  public static void sendMessageToSpecificAutobahn(Address addr, String pubTopic, byte[] message) {
    if (autobahn == null) {
      return;
    }

    autobahn.publishSpecific(addr, pubTopic, message);
  }

  /*
   * @note only 1 thing at a time can subscribe to a topic!
   */
  public static void subscribeAutobahn(String sub, Consumer<byte[]> callback) {
    if (autobahn == null) {
      return;
    }

    autobahn.subscribe(sub, callback);
  }

  public static void unsubscribeAutobahn(String sub) {
    if (autobahn == null) {
      return;
    }

    autobahn.unsubscribe(sub);
  }
}
