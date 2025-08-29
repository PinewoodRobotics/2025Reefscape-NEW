package frc.robot.subsystems.background;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import autobahn.client.NamedCallback;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.BotConstants.Mode;
import frc.robot.constants.PiConstants;
import proto.util.Position.Position2d;
import proto.util.Position.RobotPosition;
import proto.util.Vector.Vector2;

@AutoLog()
public class GlobalPosition extends SubsystemBase {
  private static Pose2d lastEstimatedRobotPose = new Pose2d();
  private static long lastTimeMs;
  private static double confidence;
  private static GlobalPosition self;

  public static GlobalPosition GetInstance(Mode botMode) {
    if (self == null) {
      self = new GlobalPosition();
    }

    return self;
  }

  public static GlobalPosition GetInstance() {
    return GetInstance(null);
  }

  public static void Initialize() {
    Robot.communication.subscribe(PiConstants.AutobahnConfig.poseSubscribeTopic,
        NamedCallback.FromConsumer(GlobalPosition::subscription)).join();
  }

  private static void subscription(byte[] data) {
    RobotPosition position = null;

    try {
      position = RobotPosition.parseFrom(data);
    } catch (InvalidProtocolBufferException e) {
      System.err.println("Invalid protocol buffer exception GlobalPosition.subscription()");
      e.printStackTrace();
      return;
    }

    assert position != null;

    lastTimeMs = (long) position.getTimestamp();

    lastEstimatedRobotPose = fromPosition2d(position.getPosition2D());
    confidence = position.getConfidence();
  }

  private static Pose2d fromPosition2d(Position2d position) {
    Vector2 translation = position.getPosition();
    Vector2 direction = position.getDirection();
    return new Pose2d(new Translation2d(translation.getX(),
        translation.getY()), new Rotation2d(direction.getX(), direction.getY()));
  }

  public static Pose2d Get() {
    return lastEstimatedRobotPose;
  }

  public static Rotation2d Rotation() {
    return lastEstimatedRobotPose.getRotation();
  }

  public static Translation2d Translation() {
    return lastEstimatedRobotPose.getTranslation();
  }

  public static long Time() {
    return lastTimeMs;
  }

  public static double Confidence() {
    return confidence;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("global/position", lastEstimatedRobotPose);
  }
}
