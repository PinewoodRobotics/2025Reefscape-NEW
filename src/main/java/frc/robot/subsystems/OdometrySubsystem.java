package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.AHRSGyro;
import proto.sensor.GeneralSensorDataOuterClass.GeneralSensorData;
import proto.sensor.Odometry.OdometryData;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;
import pwrup.frc.core.proto.IDataClass;

public class OdometrySubsystem extends SubsystemBase implements IDataClass {

  private static OdometrySubsystem self;
  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  public Pose2d latestPosition;

  public static OdometrySubsystem GetInstance() {
    if (self == null) {
      self = new OdometrySubsystem();
    }

    return self;
  }

  public OdometrySubsystem() {
    this.swerve = SwerveSubsystem.GetInstance();
    this.gyro = AHRSGyro.GetInstance();

    this.odometry = new SwerveDriveOdometry(
        swerve.getKinematics(),
        getGlobalGyroRotation(),
        swerve.getSwerveModulePositions(),
        new Pose2d(5, 5, new Rotation2d()));
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
    Logger.recordOutput("odometry/pos", latestPosition);
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var all = GeneralSensorData.newBuilder().setOdometry(OdometryData.newBuilder());
    all.setSensorId("odom");

    var rotation = Vector2.newBuilder().setX((float) latestPosition.getRotation().getCos())
        .setY((float) latestPosition.getRotation().getSin());
    var pose = Position2d.newBuilder()
        .setPosition(Vector2.newBuilder().setX((float) latestPosition.getX()).setY((float) latestPosition.getY()))
        .setDirection(rotation);

    var velocity = Vector2.newBuilder().setX((float) SwerveSubsystem.GetInstance().getChassisSpeeds().vxMetersPerSecond)
        .setY((float) SwerveSubsystem.GetInstance().getChassisSpeeds().vyMetersPerSecond);

    all.setOdometry(OdometryData.newBuilder().setPosition(pose).setVelocity(velocity).build());

    return all.build().toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return "robot/odometry";
  }
}
