package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AlignmentPoints;

public class AlignmentConstants {
  public static final AlignmentPoints.AlignmentMap POINTS = new AlignmentPoints.AlignmentMap()
      .category("Coral", new AlignmentPoints.CategoryBuilder()
          .subCategory("Front", new AlignmentPoints.SubCategoryBuilder()
              .point("Left", 3.13, 3.89, 0)
              .point("Right", 3.13, 4.20, 0)));

  static {
    AlignmentPoints.setKFieldWidth(6.0);
    AlignmentPoints.setKFieldLength(6.0);
    AlignmentPoints.setPoints(POINTS);
  }

  public static class Coral {
    public static final Pose2d left = new Pose2d(0.6, -0.16, Rotation2d.fromDegrees(180)); // calculated left side of
                                                                                           // the coral
    public static final Pose2d right = new Pose2d(0.6, 0.15, Rotation2d.fromDegrees(180)); // calculated right side of
                                                                                           // the coral

    public static final Pose2d center = new Pose2d(0.7, 0, Rotation2d.fromDegrees(180)); // the center. The tag is being
    // detected relative from the
    // center of the tag.

    public static final Pose2d rightFront = new Pose2d(3.13, 3.89, new Rotation2d(0));
    public static final Pose2d leftFront = new Pose2d(3.13, 4.20, new Rotation2d(0));
  }
}
