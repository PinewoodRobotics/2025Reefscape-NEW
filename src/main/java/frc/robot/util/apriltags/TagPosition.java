package frc.robot.util.apriltags;

import edu.wpi.first.math.geometry.Pose2d;

public class TagPosition {

  public Pose2d pose;
  public int tagNumber;

  public TagPosition(Pose2d pose, int tagNumber) {
    this.pose = pose;
    this.tagNumber = tagNumber;
  }
}
