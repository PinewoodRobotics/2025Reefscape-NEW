package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {
  public static class Coral {
    // 0.48cm
    public static final Pose2d left = new Pose2d(0.6, -0.13, Rotation2d.fromDegrees(180));
    // public static final Pose2d left = new Pose2d(0.6, 0.23, Rotation2d.fromDegrees(180));
    public static final Pose2d right = new Pose2d(0.6, 0.13, Rotation2d.fromDegrees(180));
  }
}
