package frc.robot.subsystems.swerve;

import com.google.protobuf.InvalidProtocolBufferException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Communicator;
import org.pwrup.util.Vec2;
import proto.util.Position.Position2d;

public class DriveToGoal extends SubsystemBase {

  private final Swerve swerve;
  private final double minDistanceDifference;
  private Position2d lastKnownPosition, goalPosition;
  private double lastPositionUpdateTime;
  private boolean isOnline;
  private String gotoSubscribeTopic;

  public DriveToGoal(
    Swerve swerve,
    double minDistanceDifference,
    String positionSubscribeTopic,
    String gotoSubscribeTopic
  ) {
    this.swerve = swerve;
    this.minDistanceDifference = minDistanceDifference;
    this.gotoSubscribeTopic = gotoSubscribeTopic;

    Communicator.subscribeAutobahn(
      positionSubscribeTopic,
      this::positionUpdateCallback
    );
  }

  public void setOnline(boolean isOnline) {
    this.isOnline = isOnline;

    if (!isOnline) {
      goalPosition = null;
      Communicator.unsubscribeAutobahn(gotoSubscribeTopic);
    } else {
      Communicator.subscribeAutobahn(
        gotoSubscribeTopic,
        this::gotoUpdateCallback
      );
    }
  }

  public void switchOnline() {
    setOnline(!isOnline);
  }

  public void setGoal(Position2d position) {
    goalPosition = position;
  }

  public void tick() {
    if (
      !isOnline ||
      lastKnownPosition == null ||
      goalPosition == null ||
      System.currentTimeMillis() - lastPositionUpdateTime > 1000
    ) {
      return;
    }

    double distance = getDistance(lastKnownPosition, goalPosition);
    if (distance < minDistanceDifference) {
      System.out.println("Reached goal! Stopping...");
      this.setOnline(false);
      return;
    }

    Vec2 direction = fromPosition2d(goalPosition)
      .subtract(fromPosition2d(lastKnownPosition));
    direction.scaleToModulo(1);

    swerve.drive(direction, 0.0, 0.2, 0.0);
  }

  private void positionUpdateCallback(byte[] data) {
    try {
      Position2d position = Position2d.parseFrom(data);
      lastKnownPosition = position;
      lastPositionUpdateTime = System.currentTimeMillis();
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  private void gotoUpdateCallback(byte[] data) {
    try {
      Position2d position = Position2d.parseFrom(data);
      setGoal(position);
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }

  private double getDistance(Position2d position1, Position2d position2) {
    return Math.sqrt(
      Math.pow(
        position2.getPosition().getX() - position1.getPosition().getX(),
        2
      ) +
      Math.pow(
        position2.getPosition().getY() - position1.getPosition().getY(),
        2
      )
    );
  }

  private Vec2 fromPosition2d(Position2d position) {
    return new Vec2(
      position.getPosition().getX(),
      position.getPosition().getY()
    );
  }
}
