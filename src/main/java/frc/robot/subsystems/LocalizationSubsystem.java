package frc.robot.subsystems;

import com.google.protobuf.InvalidProtocolBufferException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LocalizationConstants;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.interfaces.IDataSubsystem;
import frc.robot.util.interfaces.IGyroscopeLike;
import frc.robot.util.position.RobotPosition2d;
import proto.RobotPositionOuterClass.RobotPosition;

/**
 * @purpose essentially a way to both get the robot position from anywhere in the robot while also serving as a way
 * to periodically update the robot position based on odometry data
 */
public class LocalizationSubsystem
    extends SubsystemBase
    implements IDataSubsystem {

  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  private boolean recievedFirstMsg = false;
  private Pose2d latestLocalRobotPose = null;

  private static RobotPosition2d lastEstimatedPosition;
  private static long lastTimePoint;
  private static double positionConfidence;
  private static String subTopic;

  public LocalizationSubsystem(SwerveSubsystem swerve, IGyroscopeLike gyro) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.odometry = new SwerveDriveOdometry(
        swerve.getKinematics(),
        Rotation2d.fromDegrees(this.gyro.getYaw()),
        swerve.getSwerveModulePositions(),
        new Pose2d(0, 0, new Rotation2d()));
  }

  public void setOdometryPosition(Pose2d newPose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(-this.gyro.getYaw()),
        swerve.getSwerveModulePositions(),
        newPose);
  }

  @Override
  public String getPublishTopic() {
    return LocalizationConstants.kPosPublicationTopic;
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    if (!recievedFirstMsg) {
      return null;
    }

    var speeds = swerve.getChassisSpeeds();
    var pose = odometry.getPoseMeters();

    // System.out.println(pose.getX() + " | " + pose.getY());

    var speedsSwerve = new RobotPosition2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        new Rotation2d());

    // System.out.println(speedsSwerve.getY());

    return null; /*Odometry
                 .newBuilder()
                 .setPosition(
                 Position2d
                 .newBuilder()
                 .setPosition(
                    Vector2
                        .newBuilder()
                        .setX((float) pose.getX())
                        .setY((float) pose.getY())
                        .build())
                 .setDirection(
                    Vector2
                        .newBuilder()
                        .setX((float) speedsSwerve.getX())
                        .setY((float) speedsSwerve.getY())
                        .build())
                 .build())
                 .setRotationRads((float) pose.getRotation().getRadians())
                 .setTimestamp(System.currentTimeMillis())
                 .build()
                 .toByteArray();*/
  }

  @Override
  public void periodic() {
    RobotPosition2d position = getPose2d();
    if (position != null && !recievedFirstMsg) {
      recievedFirstMsg = true;
      setOdometryPosition(position);
      System.out.println("!!!!!");
    }

    if (latestLocalRobotPose != null) {
      if (position == null) {
        return;
      }

      var distance = position
          .getTranslation()
          .getDistance(latestLocalRobotPose.getTranslation());
      if (distance > LocalizationConstants.kMaxDistanceDiffBeforeReset ||
          CustomMath.angleDifference180(
              position.getRotation().getDegrees(),
              latestLocalRobotPose.getRotation().getDegrees()) > LocalizationConstants.kMaxDegDiffBeforeReset) {
        System.out.println("Reset Odometry Position");
        setOdometryPosition(position);
      }
    }

    var positions = swerve.getSwerveModulePositions();
    var rawRotation = Rotation2d.fromDegrees(-this.gyro.getYaw());
    latestLocalRobotPose = odometry.update(rawRotation, positions);
  }

  // ---------------------------------------------

  public static void launch(String posePublishTopic) {
    Communicator.subscribeAutobahn(
        posePublishTopic,
        LocalizationSubsystem::onMessage);
    LocalizationSubsystem.subTopic = posePublishTopic;
  }

  public static void stop() {
    Communicator.unsubscribeAutobahn(LocalizationSubsystem.subTopic);
  }

  public static RobotPosition2d getPose2d() {
    return LocalizationSubsystem.lastEstimatedPosition;
  }

  public static double getPosConfidence() {
    return LocalizationSubsystem.positionConfidence;
  }

  public static long getTimePoint() {
    return LocalizationSubsystem.lastTimePoint;
  }

  private static void onMessage(byte[] data) {
    // System.out.println("!!!!!!");
    try {
      var position = RobotPosition.parseFrom(data);
      LocalizationSubsystem.lastTimePoint = (long) position.getTimestamp();
      LocalizationSubsystem.lastEstimatedPosition = new RobotPosition2d(
          position.getEstimatedPosition().getPosition().getX(),
          position.getEstimatedPosition().getPosition().getY(),
          new Rotation2d(
              position.getEstimatedPosition().getDirection().getX(),
              position.getEstimatedPosition().getDirection().getY()));
      LocalizationSubsystem.positionConfidence = position.getConfidence();
    } catch (InvalidProtocolBufferException e) {
      e.printStackTrace();
    }
  }
}
