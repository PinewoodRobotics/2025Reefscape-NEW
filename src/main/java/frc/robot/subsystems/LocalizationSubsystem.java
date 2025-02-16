package frc.robot.subsystems;

import com.google.protobuf.InvalidProtocolBufferException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LocalizationConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Communicator;
import frc.robot.util.interfaces.IDataSubsystem;
import frc.robot.util.interfaces.IGyroscopeLike;
import frc.robot.util.position.PosExtrapolatorPose2d;
import proto.OdometryOuterClass.Odometry;
import proto.RobotPositionOuterClass.RobotPosition;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;

/**
 * @purpose essentially a way to both get the robot position from anywhere in the robot while also serving as a way
 * to periodically update the robot position based on odometry data 
 */
public class LocalizationSubsystem extends SubsystemBase implements IDataSubsystem {
  private final SwerveSubsystem swerve;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  private double gyroOffset = 0;

  private static PosExtrapolatorPose2d lastEstimatedPosition;
  private static long lastTimePoint;
  private static double positionConfidence;
  private static String subTopic;

  public LocalizationSubsystem(SwerveSubsystem swerve, IGyroscopeLike gyro) {
    this.swerve = swerve;
    this.kinematics = new SwerveDriveKinematics(
        SwerveConstants.frontLeftTranslation,
        SwerveConstants.frontRightTranslation,
        SwerveConstants.rearLeftTranslation,
        SwerveConstants.rearRightTranslation);
    this.gyro = gyro;
    this.odometry = new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(this.getGlobalGyroAngle()),
        swerve.getSwerveModulePositions(),
        new Pose2d(0, 0, new Rotation2d()));
  }

  public void setGyroOffset(double offset) {
    gyroOffset = offset;
  }

  public void setOdometryPosition(Pose2d newPose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(getGlobalGyroAngle()),
        swerve.getSwerveModulePositions(),
        newPose);
  }

  private double getGlobalGyroAngle() {
    return gyro.getYaw() + gyroOffset;
  }

  @Override
  public String getPublishTopic() {
    return LocalizationConstants.kPosPublicationTopic;
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var speeds = kinematics.toChassisSpeeds(swerve.getSwerveModuleStates());
    var pose = odometry.getPoseMeters();
    return Odometry
        .newBuilder()
        .setPosition(
            Position2d
                .newBuilder()
                .setPosition(
                    Vector2
                        .newBuilder()
                        .setX(-(float) pose.getX())
                        .setY(-(float) pose.getY())
                        .build())
                .setDirection(
                    Vector2
                        .newBuilder()
                        .setX((float) speeds.vxMetersPerSecond)
                        .setY((float) speeds.vyMetersPerSecond)
                        .build())
                .build())
        .setRotation(
            Vector2
                .newBuilder()
                .setX((float) pose.getRotation().getSin())
                .setY((float) pose.getRotation().getCos()))
        .setTimestamp(System.currentTimeMillis())
        .build()
        .toByteArray();
  }

  @Override
  public void periodic() {
    var positions = swerve.getSwerveModulePositions();
    var latestRobotPos = odometry.update(Rotation2d.fromDegrees(getGlobalGyroAngle()), positions);

    var position = getPose2d();
    if (position == null || getTimePoint() > LocalizationConstants.kMaxTimeMs) {
      return;
    }

    var distance = position.getTranslation().getDistance(latestRobotPos.getTranslation());
    if (distance > LocalizationConstants.kMaxDistanceDiffBeforeReset) {
      setOdometryPosition(position);
    }

    var rotationDiff = Math.abs(position.getRotation().getDegrees() - latestRobotPos.getRotation().getDegrees());
    if (rotationDiff > LocalizationConstants.kMaxDegDiffBeforeReset) {
      setGyroOffset(getGlobalGyroAngle() - position.getRotation().getDegrees());
    }
  }

  // ---------------------------------------------

  public static void launch(String posePublishTopic) {
    Communicator.subscribeAutobahn(posePublishTopic, LocalizationSubsystem::onMessage);
    LocalizationSubsystem.subTopic = posePublishTopic;
  }

  public static void stop() {
    Communicator.unsubscribeAutobahn(LocalizationSubsystem.subTopic);
  }

  public static PosExtrapolatorPose2d getPose2d() {
    return LocalizationSubsystem.lastEstimatedPosition;
  }

  public static double getPosConfidence() {
    return LocalizationSubsystem.positionConfidence;
  }

  public static long getTimePoint() {
    return LocalizationSubsystem.lastTimePoint;
  }

  private static void onMessage(byte[] data) {
    try {
      var position = RobotPosition.parseFrom(data);
      LocalizationSubsystem.lastTimePoint = (long) position.getTimestamp();
      LocalizationSubsystem.lastEstimatedPosition = new PosExtrapolatorPose2d(
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
