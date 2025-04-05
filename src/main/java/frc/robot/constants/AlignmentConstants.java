package frc.robot.constants;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.SlowdownConfig;
import frc.robot.util.config.TagConfig;

public class AlignmentConstants {

  public static final Distance kElevatorRaiseThreshold = Distance.ofRelativeUnits(
    0.75,
    Meter
  );
  public static final long tagTimeThreshhold = 500;
  public static final DriveConfig kDriveConfig = new DriveConfig(
    0.02,
    2,
    0.1,
    0.3
  );
  public static final DriveConfig kDriveConfigAuton = new DriveConfig(
    0.02,
    2,
    0.4,
    0.3
  );
  public static final SlowdownConfig kSlowdownConfig = new SlowdownConfig(
    1.5,
    0.5,
    0.5,
    0.5,
    0.2
  );
  public static final TagConfig kTagConfigTesting = new TagConfig(500, 9);

  public static final Pose2d poleLeft = new Pose2d(
    -0.5,
    0.11,
    new Rotation2d(0)
  );

  public static final Pose2d poleRight = new Pose2d(
    -0.5,
    -0.11,
    new Rotation2d(0)
  );

  public static final Pose2d autonDriveForwardRightSide = new Pose2d(
    -2.24,
    0,
    new Rotation2d(0)
  );

  public static final Pose2d autonDriveForwardLeftSide = new Pose2d(
    -2.24,
    0,
    new Rotation2d(0)
  );

  public static final Pose2d autonDriveForwardCenter = new Pose2d(
    2,
    0.5,
    new Rotation2d(0)
  );
}
