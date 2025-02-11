package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Autobahn;
import frc.robot.util.Communicator;
import java.util.ArrayList;
import java.util.List;

public class PublicationSubsystem extends SubsystemBase {

  private final List<IDataSubsystem> dataSubsystems = new ArrayList<>();

  public void addDataSubsystem(IDataSubsystem... dataSubsystem) {
    dataSubsystems.addAll(List.of(dataSubsystem));
  }

  @Override
  public void periodic() {
    for (IDataSubsystem dataSubsystem : dataSubsystems) {
      Communicator.sendMessageAutobahn(
        dataSubsystem.getPublishTopic(),
        dataSubsystem.getRawConstructedProtoData()
      );
    }
  }
}
