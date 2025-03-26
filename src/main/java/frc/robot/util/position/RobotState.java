package frc.robot.util.position;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotState {
  public Pose2d position;
  boolean futureBall, futureTube;

  public RobotState(Pose2d position, boolean futureBall, boolean futureTube) {
    this.position = position;
    this.futureBall = futureBall;
    this.futureTube = futureTube;
  }
}
