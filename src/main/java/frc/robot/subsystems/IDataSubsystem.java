package frc.robot.subsystems;

public interface IDataSubsystem {
  public byte[] getRawConstructedProtoData();

  public String getPublishTopic();
}
