package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.PiConstants;
import proto.util.Position.RobotPosition;

public class GlobalPosition extends SubsystemBase {
  private static GlobalPosition self;
  private static Pose2d position;

  public static GlobalPosition GetInstance() {
    if (self == null) {
      self = new GlobalPosition();
    }
    return self;
  }

  public GlobalPosition() {
    Robot.getAutobahnClient().subscribe(PiConstants.AutobahnConfig.poseSubscribeTopic,
        NamedCallback.FromConsumer(this::subscription));
  }

  public void subscription(byte[] payload) {
    try {
      RobotPosition position = RobotPosition.parseFrom(payload);
      var pose = position.getPosition2D().getPosition();
      var direction = position.getPosition2D().getDirection();

      GlobalPosition.position = new Pose2d(pose.getX(),
          pose.getY(),
          new Rotation2d(direction.getX(), direction.getY()));
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
      return;
    }
  }

  public static Pose2d Get() {
    return position;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Global/pose", position);
  }
}
