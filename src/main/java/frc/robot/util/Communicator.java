package frc.robot.util;

import com.google.gson.Gson;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.online.Autobahn;
import java.util.function.Consumer;
import org.pwrup.util.IPublisher;

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

  /**
   * @apiNote Please note that if the "init" function is not called before this function call, the output will be nothing and the request will just be eaten up and nothing will happen.
   */
  public static void sendMessageAutobahn(String pubTopic, byte[] message) {
    if (autobahn == null) {
      return;
    }

    autobahn.publish(pubTopic, message);
  }

  /**
   * @apiNote Please note that if the "init" function is not called before this function call, the output will be nothing and the request will just be eaten up and nothing will happen.
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
