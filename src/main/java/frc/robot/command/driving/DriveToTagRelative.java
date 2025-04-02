package frc.robot.command.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.apriltags.TagPosition;
import org.pwrup.util.Vec2;
import proto.RobotPositionOuterClass.RobotPosition;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;

public class DriveToTagRelative extends Command {

  public static class DriveConfig {

    final double translationStoppingDistance;
    final double angularStoppingDistanceDeg;
    final double maxRotationSpeed;
    final double maxSpeed;
    final double secondTierDistance;
    final double thirdTierDistance;
    final double firstTierMaxSpeedMultiplier;
    final double secondTierMaxSpeedMultiplier;
    final double thirdTierMaxSpeedMultiplier;
    final long maxTimeNoTagSeen;

    public DriveConfig(
      double translationStoppingDistance,
      double angularStoppingDistanceDeg,
      double maxRotationSpeed,
      double maxSpeed,
      double secondTierDistance,
      double thirdTierDistance,
      double firstTierMaxSpeedMultiplier,
      double secondTierMaxSpeedMultiplier,
      double thirdTierMaxSpeedMultiplier,
      long maxTimeNoTagSeen
    ) {
      this.translationStoppingDistance = translationStoppingDistance;
      this.angularStoppingDistanceDeg = angularStoppingDistanceDeg;
      this.maxRotationSpeed = maxRotationSpeed;
      this.maxSpeed = maxSpeed;
      this.secondTierDistance = secondTierDistance;
      this.thirdTierDistance = thirdTierDistance;
      this.firstTierMaxSpeedMultiplier = firstTierMaxSpeedMultiplier;
      this.secondTierMaxSpeedMultiplier = secondTierMaxSpeedMultiplier;
      this.thirdTierMaxSpeedMultiplier = thirdTierMaxSpeedMultiplier;
      this.maxTimeNoTagSeen = maxTimeNoTagSeen;
    }
  }

  private final SwerveSubsystem m_swerveSubsystem;
  private final DriveConfig m_config;
  private final Pose2d m_targetPose;
  private final int m_tagNumber;

  private boolean m_isDone = false;
  private long m_lastTimeTagSeen = 0;

  public DriveToTagRelative(
    SwerveSubsystem swerveSubsystem,
    Pose2d targetPose,
    int tagNumber,
    DriveConfig config,
    boolean isDone
  ) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_targetPose = targetPose;
    this.m_tagNumber = tagNumber;
    this.m_config = config;
    setIsDone(isDone);
    addRequirements(m_swerveSubsystem);
  }

  public void setIsDone(boolean isDone) {
    this.m_isDone = isDone;
    if (!isDone) {
      this.m_lastTimeTagSeen = System.currentTimeMillis();
    }
  }

  @Override
  public void execute() {
    if (m_isDone) {
      return;
    }

    TagPosition tagPosition = findValidTagPosition();
    if (tagPosition == null) {
      if (
        System.currentTimeMillis() -
        m_lastTimeTagSeen >
        m_config.maxTimeNoTagSeen
      ) {
        end(true);
      }

      return;
    }

    m_lastTimeTagSeen = System.currentTimeMillis();
    updateRobotPosition(tagPosition);
  }

  private TagPosition findValidTagPosition() {
    var recentTags = AprilTagSubsystem.getLatestTagPositions();

    for (var tags : recentTags) {
      TagPosition position = tags.positions
        .stream()
        .filter(tag -> tag.tagNumber == m_tagNumber)
        .findFirst()
        .orElse(null);

      if (
        position != null &&
        System.currentTimeMillis() - tags.timestamp < m_config.maxTimeNoTagSeen
      ) {
        return position;
      }
    }
    return null;
  }

  private void updateRobotPosition(TagPosition tagPosition) {
    var finalPose = calculateFinalPose(tagPosition.pose);
    var distance = finalPose.getTranslation().getNorm();
    var direction = calculateDirectionVector(finalPose);
    var rotationDirection = calculateRotationDirection(finalPose);
    var speed = calculateSpeed(distance);

    m_swerveSubsystem.driveRaw(
      direction,
      m_config.maxRotationSpeed * rotationDirection,
      speed
    );
    sendPositionUpdate(finalPose, tagPosition.pose);

    if (
      distance < m_config.translationStoppingDistance && rotationDirection == 0
    ) {
      end(false);
    }
  }

  private Pose2d calculateFinalPose(Pose2d tagPose) {
    return new Pose2d(tagPose.toMatrix().times(m_targetPose.toMatrix()));
  }

  private Vec2 calculateDirectionVector(Pose2d finalPose) {
    return new Vec2((float) finalPose.getX(), (float) -finalPose.getY())
      .scaleToModulo(1);
  }

  private int calculateRotationDirection(Pose2d finalPose) {
    double diff = finalPose.getRotation().getRadians();
    if (diff > Math.toRadians(m_config.angularStoppingDistanceDeg)) {
      return -1;
    } else if (diff < -Math.toRadians(m_config.angularStoppingDistanceDeg)) {
      return 1;
    }
    return 0;
  }

  private double calculateSpeed(double distance) {
    double speed = m_config.maxSpeed * m_config.firstTierMaxSpeedMultiplier;
    if (distance < m_config.secondTierDistance) {
      speed = m_config.maxSpeed * m_config.secondTierMaxSpeedMultiplier;
    }
    if (distance < m_config.thirdTierDistance) {
      speed = m_config.maxSpeed * m_config.thirdTierMaxSpeedMultiplier;
    }
    return speed;
  }

  private void sendPositionUpdate(Pose2d finalPose, Pose2d tagPose) {
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
                .build()
            )
            .setDirection(
              Vector2
                .newBuilder()
                .setX((float) tagPose.getRotation().getCos())
                .setY((float) tagPose.getRotation().getSin())
                .build()
            )
            .build()
        )
        .build()
        .toByteArray()
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_isDone = true;
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }
}
