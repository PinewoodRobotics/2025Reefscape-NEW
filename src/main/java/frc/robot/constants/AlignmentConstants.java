package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AlignmentPoints;

public class AlignmentConstants {
  public static class Coral {
    public static final Pose2d rightFront = new Pose2d(3.13, 3.89, new Rotation2d(0));
    public static final Pose2d leftFront = new Pose2d(3.13, 4.20, new Rotation2d(0));

    public static final Pose2d left = new Pose2d(0, -0.155, new Rotation2d(0));
    public static final Pose2d right = new Pose2d(0, 0.155, new Rotation2d(0));
  }

  public static final AlignmentPoints.AlignmentMap POINTS = new AlignmentPoints.AlignmentMap()
      .category("Coral", new AlignmentPoints.CategoryBuilder()
          .subCategory("Front", new AlignmentPoints.SubCategoryBuilder()
              .point("Left", 3.13, 4.18, 0)
              .point("Right", 3.13, 3.9, 0)));

  public static AlignmentPoints.SubCategoryBuilder GetCoral(Pose2d centerHeading) {
    var builder = new AlignmentPoints.SubCategoryBuilder();
    // builder.point("Left", Coral.left.plus(centerHeading));
    // builder.point("Right", Coral.right.plus(centerHeading));
    return builder;
  }

  static {
    AlignmentPoints.setKFieldWidth(6.0);
    AlignmentPoints.setKFieldLength(6.0);
    AlignmentPoints.setPoints(POINTS);
  }
}
