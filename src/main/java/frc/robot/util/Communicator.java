package frc.robot.util;

import com.google.gson.Gson;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.Autobahn;
import java.util.function.Consumer;
import org.pwrup.util.IPublisher;

public class Communicator implements IPublisher {

  private final Gson gson = new Gson();
  private static Autobahn autobahn;

  @Override
  public void publish(String arg0, Object arg1, Class<?> arg2) {
    var networkTable = NetworkTableInstance.getDefault();
    // System.out.println(gson.toJson(arg1, arg2));
    networkTable.getEntry(arg0).setString(gson.toJson(arg1, arg2));
  }

  public static void init(Autobahn autobahn) {
    Communicator.autobahn = autobahn;
  }

  public static void sendMessageAutobahn(String pubTopic, byte[] message) {
    if (autobahn == null) {
      throw new RuntimeException("Autobahn not initialized");
    }

    autobahn.publish(pubTopic, message);
  }

  /*
   * @note only 1 thing at a time can subscribe to a topic!
   */
  public static void subscribeAutobahn(String sub, Consumer<byte[]> callback) {
    if (autobahn == null) {
      throw new RuntimeException("Autobahn not initialized");
    }

    autobahn.subscribe(sub, callback);
  }

  public static void unsubscribeAutobahn(String sub) {
    if (autobahn == null) {
      throw new RuntimeException("Autobahn not initialized");
    }

    autobahn.unsubscribe(sub);
  }
}
