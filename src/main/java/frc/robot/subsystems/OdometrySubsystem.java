package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Communicator;
import frc.robot.util.interfaces.IGyroscopeLike;
import proto.RobotPositionOuterClass.RobotPosition;
import proto.util.Position.Position2d;
import proto.util.Vector.Vector2;

public class OdometrySubsystem extends SubsystemBase {

  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry odometry;
  private final IGyroscopeLike gyro;
  public Pose2d latestPosition;

  public OdometrySubsystem(SwerveSubsystem swerve, IGyroscopeLike gyro) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.odometry =
      new SwerveDriveOdometry(
        swerve.getKinematics(),
        Rotation2d.fromDegrees(this.gyro.getYaw()),
        swerve.getSwerveModulePositions(),
        new Pose2d(0, 0, new Rotation2d())
      );
  }

  public void setOdometryPosition(Pose2d newPose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(-this.gyro.getYaw()),
      swerve.getSwerveModulePositions(),
      newPose
    );
  }

  @Override
  public void periodic() {
    var positions = swerve.getSwerveModulePositions();
    var rawRotation = Rotation2d.fromDegrees(-this.gyro.getYaw());

    latestPosition = odometry.update(rawRotation, positions);
    /*Communicator.sendMessageAutobahn(
      "pos-extrapolator/robot-position",
      RobotPosition
        .newBuilder()
        .setEstimatedPosition(
          Position2d
            .newBuilder()
            .setPosition(
              Vector2
                .newBuilder()
                .setX((float) latestPosition.getX())
                .setY((float) latestPosition.getY())
                .build()
            )
            .setDirection(
              Vector2
                .newBuilder()
                .setX((float) latestPosition.getRotation().getCos())
                .setY((float) latestPosition.getRotation().getSin())
                .build()
            )
            .build()
        )
        .build()
        .toByteArray()
    );*/
  }
}
