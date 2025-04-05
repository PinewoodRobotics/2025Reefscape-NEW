package frc.robot.command.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.CustomMath.DrivingMath;
import frc.robot.util.apriltags.TimedTagPosition;
import frc.robot.util.config.DriveConfig;
import frc.robot.util.config.SlowdownConfig;
import frc.robot.util.config.TagConfig;
import org.pwrup.util.Vec2;
import proto.RobotPositionOuterClass.RobotPosition;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;

public class OdomAssistedTagAlignment extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final OdometrySubsystem m_odometrySubsystem;
  private final Pose2d targetPose;
  private final DriveConfig driveConfig;
  private final SlowdownConfig slowdownConfig;
  private final boolean isOdomAssisted;
  private long startTime;
  private TagConfig tagConfig;
  private double latestDistance = Double.POSITIVE_INFINITY;

  private boolean isDone = false;

  public OdomAssistedTagAlignment(
    SwerveSubsystem swerveSubsystem,
    OdometrySubsystem odometrySubsystem,
    Pose2d targetPose,
    DriveConfig driveConfig,
    TagConfig tagConfig,
    SlowdownConfig slowdownConfig,
    boolean isOdomAssisted,
    boolean isDone
  ) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_odometrySubsystem = odometrySubsystem;
    this.targetPose = targetPose;
    this.isOdomAssisted = isOdomAssisted;

    this.tagConfig = tagConfig;
    this.driveConfig = driveConfig;
    this.slowdownConfig = slowdownConfig;

    setIsDone(isDone);

    addRequirements(m_swerveSubsystem, odometrySubsystem);
  }

  public double getLatestDistance() {
    return latestDistance;
  }

  @Override
  public void initialize() {
    isDone = false;
    latestDistance = Double.POSITIVE_INFINITY;

    tagConfig =
      new TagConfig(
        tagConfig.getMaxTimeNoTagSeen(),
        AprilTagSubsystem.closestTagCurrently(
          AlignmentConstants.tagTimeThreshhold
        )
      );
  }

  public void setIsDone(boolean isDone) {
    this.isDone = isDone;
    if (!isDone) {
      this.startTime = System.currentTimeMillis();
    }
  }

  @Override
  public void execute() {
    var tagPosition = AprilTagSubsystem.getLatestTagPosition(
      tagConfig.getTagId()
    );

    if (tagPosition == null) {
      if (
        System.currentTimeMillis() - startTime > tagConfig.getMaxTimeNoTagSeen()
      ) {
        isDone = true;
      }

      return;
    } else if (
      System.currentTimeMillis() -
      tagPosition.getTimestamp() >
      tagConfig.getMaxTimeNoTagSeen()
    ) {
      if (isOdomAssisted) {
        /*new Pose2d(
              m_odometrySubsystem.latestPosition
                .toMatrix()
                .inv()
                .times(tagPosition.getPose().toMatrix())
            ) */
        tagPosition =
          new TimedTagPosition(
            CustomMath.fromTransformationMatrix2dToPose2d(
              CustomMath
                .fromPose2dToMatrix(m_odometrySubsystem.latestPosition)
                .invert()
                .mult(CustomMath.fromPose2dToMatrix(tagPosition.getPose()))
            ),
            tagPosition.getTagNumber(),
            System.currentTimeMillis()
          );
      } else {
        isDone = true;
      }
    } else {
      m_odometrySubsystem.setOdometryPosition(new Pose2d());
    }

    updateRobotPosition(tagPosition);
  }

  private void updateRobotPosition(TimedTagPosition tagPosition) {
    var finalPose = calculateFinalPose(tagPosition.getPose());
    var distance = finalPose.getTranslation().getNorm();
    latestDistance = distance;
    var direction = DrivingMath.calculateDirectionVector(finalPose);
    var rotationDirection = DrivingMath.calculateRotationDirection(
      finalPose,
      driveConfig
    );
    var speed = DrivingMath.calculateSpeed(
      distance,
      driveConfig,
      slowdownConfig
    );

    var rotation = driveConfig.getMaxRotationSpeed() * rotationDirection;

    if (distance < driveConfig.getTranslationStoppingDistance()) {
      direction = new Vec2(0, 0);
    }

    m_swerveSubsystem.driveRaw(direction, rotation, speed);

    sendPositionUpdate(finalPose, tagPosition.getPose());

    if (
      distance < driveConfig.getTranslationStoppingDistance() &&
      rotationDirection == 0
    ) {
      isDone = true;
    }
  }

  /*
  private Pose2d calculateFinalPose(Pose2d tagPose) {
    return new Pose2d(tagPose.toMatrix().times(targetPose.toMatrix()));
  }
     */

  private Pose2d calculateFinalPose(Pose2d tagPose) {
    return CustomMath.fromTransformationMatrix2dToPose2d(
      CustomMath
        .fromPose2dToMatrix(tagPose)
        .mult(CustomMath.fromPose2dToMatrix(targetPose))
    );
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
  public boolean isFinished() {
    return isDone;
  }

  @Override
  public void end(boolean interrupted) {
    isDone = true;
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }
}
