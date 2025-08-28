package frc.robot.subsystems.background;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import pwrup.frc.core.proto.IDataClass;

public class PublicationSubsystem extends SubsystemBase {

  private static List<IDataClass> registry = new ArrayList<>();

  public static PublicationSubsystem self;

  public static PublicationSubsystem GetInstance() {
    if (self == null) {
      self = new PublicationSubsystem();
    }

    return self;
  }

  public static void register(IDataClass... rPublishers) {
    for (IDataClass p : rPublishers) {
      registry.add(p);
    }
  }

  @Override
  public void periodic() {
    var handles = new ArrayList<CompletableFuture<?>>();
    for (IDataClass p : registry) {
      var handle = Robot.communication.publish(p.getPublishTopic(), p.getRawConstructedProtoData());
      handles.add(handle);
    }

    CompletableFuture.allOf(handles.toArray(new CompletableFuture[0])).join();
  }
}
