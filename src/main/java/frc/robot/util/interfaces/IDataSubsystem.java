package frc.robot.util.interfaces;

public interface IDataSubsystem {
  public byte[] getRawConstructedProtoData();

  public String getPublishTopic();
}
