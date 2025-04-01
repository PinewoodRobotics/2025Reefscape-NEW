package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import frc.robot.util.CustomMath.EasingFunctions;
import frc.robot.util.apriltags.TagPosition;

public class DriveToTagRelative extends Command {

  private final SwerveSubsystem m_swerveSubsystem;

  private double translationStoppingDistance, angularStoppingDistanceDeg, maxRotationSpeed;
  private Pose2d tag_in_robot_wanted;
  private boolean isDone = false;
  private long lastTimeTagSeen = 0;
  private long maxTimeNoTagSeen;
  private int tagNumber;

  private boolean isTotalDistanceSet = false;
  private double totalDistance;
  private double totalAngle;

  public DriveToTagRelative(
      SwerveSubsystem swerveSubsystem,
      Pose2d finalPosition,
      int tagNumber,
      long maxTimeNoTagSeen,
      double translationStoppingDistance,
      double angularStoppingDistanceDeg,
      double maxRotationSpeed,
      boolean isDone) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.tag_in_robot_wanted = finalPosition;
    this.tagNumber = tagNumber;
    this.maxTimeNoTagSeen = maxTimeNoTagSeen;
    this.translationStoppingDistance = translationStoppingDistance;
    this.angularStoppingDistanceDeg = angularStoppingDistanceDeg;
    this.maxRotationSpeed = maxRotationSpeed;
    setIsDone(isDone);

    addRequirements(m_swerveSubsystem);
  }

  public void setIsDone(boolean isDone) {
    this.isDone = isDone;
    if (!isDone) {
      this.lastTimeTagSeen = System.currentTimeMillis();
    }
  }

  @Override
  public void execute() {
    if (isDone) {
      return;
    }

    var recentTags = AprilTagSubsystem.getLatestTagPositions();

    TagPosition position = null;
    for (var tags : recentTags) {
      position = tags.positions
          .stream()
          .filter(tag -> tag.tagNumber == tagNumber)
          .findFirst()
          .orElse(null);

      if (position != null) {
        break;
      }
    }

    if (position == null) {
      if (System.currentTimeMillis() - lastTimeTagSeen > maxTimeNoTagSeen) {
        end(true);
      }

      return;
    }

    lastTimeTagSeen = System.currentTimeMillis();

    var tag_in_robot = position.pose;
    var finalPose = CustomMath.finalPointDirection(tag_in_robot, tag_in_robot_wanted);

    var dist = finalPose.getTranslation().getNorm();
    if (!isTotalDistanceSet) {
      this.totalDistance = dist;
      this.totalAngle = finalPose.getRotation().getRadians();
      this.isTotalDistanceSet = true;
    }

    var convertedDirectionVec = new Vec2(
        (float) finalPose.getX(),
        (float) -finalPose.getY()).scaleToModulo(1);

    var rotationDirection = CustomMath.rotationDirection(finalPose.getRotation().getRadians(),
        Math.toRadians(angularStoppingDistanceDeg));

    var rotation = EasingFunctions
        .easeOutQuint(finalPose.getRotation().getRadians(), totalDistance, 0, 0.0, rotationDirection * maxRotationSpeed,
            5);

    m_swerveSubsystem.driveRaw(convertedDirectionVec, rotation, EasingFunctions
        .easeOutQuint(dist, totalDistance, 0, 0.05, 0.15, 5));

    if (dist < translationStoppingDistance && rotationDirection == 0) {
      end(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    isDone = true;
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }

  /*
   * 
   * 
   Communicator.sendMessageAutobahn(
        "pos-extrapolator/robot-position",
        RobotPosition
            .newBuilder()
            .setEstimatedPosition(
                Position2d
                    .newBuilder()
                    .setPosition(
                        Vector2
                            .newBuilder()
                            .setX((float) finalPose.getX())
                            .setY((float) finalPose.getY())
                            .build())
                    .setDirection(
                        Vector2
                            .newBuilder()
                            .setX((float) tag_in_robot.getRotation().getCos())
                            .setY((float) tag_in_robot.getRotation().getSin())
                            .build())
                    .build())
            .build()
            .toByteArray());
   */
}
