package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PiConstants;
import frc.robot.hardware.AHRSGyro;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.Odometry.OdometryData;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class OdometrySubsystem extends SubsystemBase implements IDataClass {

  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  public Pose2d latestPosition;

  public static OdometrySubsystem GetInstance() {
    return new OdometrySubsystem(SwerveSubsystem.GetInstance(), AHRSGyro.GetInstance());
  }

  public OdometrySubsystem(SwerveSubsystem swerve, IGyroscopeLike gyro) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.odometry = new SwerveDriveOdometry(
        swerve.getKinematics(),
        getGlobalGyroRotation(),
        swerve.getSwerveModulePositions(),
        new Pose2d(0, 0, new Rotation2d()));
  }

  public void setOdometryPosition(Pose2d newPose) {
    odometry.resetPosition(
        getGlobalGyroRotation(),
        swerve.getSwerveModulePositions(),
        newPose);
  }

  public Rotation2d getGlobalGyroRotation() {
    return Rotation2d.fromDegrees(-this.gyro.getYaw());
  }

  @Override
  public void periodic() {
    var positions = swerve.getSwerveModulePositions();
    latestPosition = odometry.update(getGlobalGyroRotation(), positions);
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    ChassisSpeeds speeds = swerve.getGlobalChassisSpeeds(getGlobalGyroRotation());
    Logger.recordOutput("odometry/speeds", speeds);

    return GeneralSensorData.newBuilder().setOdometry(OdometryData.newBuilder()
        .setVelocity(Vector2
            .newBuilder()
            .setX((float) speeds.vxMetersPerSecond)
            .setY((float) speeds.vyMetersPerSecond)
            .build())
        .setPosition(
            Position2d
                .newBuilder()
                .setPosition(
                    Vector2
                        .newBuilder()
                        .setX((float) latestPosition.getX())
                        .setY((float) latestPosition.getY())
                        .build())
                .setDirection(
                    Vector2
                        .newBuilder()
                        .setX((float) latestPosition.getRotation().getCos())
                        .setY((float) latestPosition.getRotation().getSin())
                        .build())
                .build())
        .build())
        .setSensorId("odom")
        .setTimestamp(System.currentTimeMillis()).build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return PiConstants.AutobahnConfig.odometryPublishTopic;
  }
}
