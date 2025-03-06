package frc.robot.util.online;

public class Address {

  private final String host;
  private final int port;

  public Address(String host, int port) {
    this.host = host;
    this.port = port;
  }

  public String makeUrl() {
    return "ws://" + host + ":" + port;
  }

  @Override
  public String toString() {
    return makeUrl();
  }
}
