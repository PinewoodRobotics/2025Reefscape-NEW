package frc.robot.util;

import org.pwrup.util.IPublisher;

import com.google.gson.Gson;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Communicator implements IPublisher {

  private final Gson gson = new Gson();

  @Override
  public void publish(String arg0, Object arg1, Class<?> arg2) {
    var networkTable = NetworkTableInstance.getDefault();
    networkTable.getEntry(arg0).setString(gson.toJson(arg1, arg2));
  }
}
