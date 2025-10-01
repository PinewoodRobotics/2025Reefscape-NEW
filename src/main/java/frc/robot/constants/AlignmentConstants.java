package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {
  public static class Coral {
    public static final Pose2d left = new Pose2d(0.6, -0.15, Rotation2d.fromDegrees(180)); // calculated left side of
                                                                                           // the coral
    public static final Pose2d right = new Pose2d(0.6, 0.17, Rotation2d.fromDegrees(180)); // calculated right side of
                                                                                           // the coral

    public static final Pose2d center = new Pose2d(0.7, 0, Rotation2d.fromDegrees(180)); // the center. The tag is being
                                                                                         // detected relative from the
                                                                                         // center of the tag.
  }
}
