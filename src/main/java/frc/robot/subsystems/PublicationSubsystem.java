package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import pwrup.frc.core.proto.IDataClass;

public class PublicationSubsystem extends SubsystemBase {

  private final List<IDataClass> dataClasses;
  private static PublicationSubsystem self;

  public static PublicationSubsystem GetInstance(IDataClass... dataClasses) {
    if (self == null) {
      self = new PublicationSubsystem(dataClasses);
    }

    return self;
  }

  public static PublicationSubsystem GetInstance() {
    return PublicationSubsystem.GetInstance(new IDataClass[] {});
  }

  public PublicationSubsystem(IDataClass... dataClasses) {
    this.dataClasses = new ArrayList<>(List.of(dataClasses));
  }

  public static void addDataClass(IDataClass dataClass) {
    GetInstance().dataClasses.add(dataClass);
  }

  public static void addDataClasses(IDataClass... dataClasses) {
    GetInstance().dataClasses.addAll(List.of(dataClasses));
  }

  @Override
  public void periodic() {
    for (var dataClass : dataClasses) {
      var data = dataClass.getRawConstructedProtoData();
      var topic = dataClass.getPublishTopic();
      Robot.getAutobahnClient().publish(topic, data);
    }
  }
}
