package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.IGyroscopeLike;
import frc.robot.hardware.RobotWheelMover;
import frc.robot.subsystems.IDataSubsystem;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import org.pwrup.SwerveDrive;
import org.pwrup.util.Config;
import org.pwrup.util.Vec2;
import org.pwrup.util.Wheel;
import proto.OdometryOuterClass.Odometry;

public class Swerve extends SubsystemBase implements IDataSubsystem {

  public final RobotWheelMover m_frontLeftSwerveModule;
  private final RobotWheelMover m_frontRightSwerveModule;
  private final RobotWheelMover m_rearLeftSwerveModule;
  private final RobotWheelMover m_rearRightSwerveModule;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final SwerveDrive swerve;
  private final IGyroscopeLike m_gyro;
  private double gyroOffset = 0;

  public Swerve(IGyroscopeLike gyro, Communicator communicator) {
    this.m_gyro = gyro;
    this.m_frontLeftSwerveModule =
      new RobotWheelMover(
        SwerveConstants.kFrontLeftDriveMotorPort,
        SwerveConstants.kFrontLeftDriveMotorReversed,
        SwerveConstants.kFrontLeftTurningMotorPort,
        SwerveConstants.kFrontLeftTurningMotorReversed,
        SwerveConstants.kFrontLeftCANcoderPort,
        SwerveConstants.kFrontLeftCANcoderDirection,
        SwerveConstants.kFrontLeftCANcoderMagnetOffset,
        "FL"
      );
    this.m_frontRightSwerveModule =
      new RobotWheelMover(
        SwerveConstants.kFrontRightDriveMotorPort,
        SwerveConstants.kFrontRightDriveMotorReversed,
        SwerveConstants.kFrontRightTurningMotorPort,
        SwerveConstants.kFrontRightTurningMotorReversed,
        SwerveConstants.kFrontRightCANcoderPort,
        SwerveConstants.kFrontRightCANcoderDirection,
        SwerveConstants.kFrontRightCANcoderMagnetOffset,
        "FR"
      );
    this.m_rearLeftSwerveModule =
      new RobotWheelMover(
        SwerveConstants.kRearLeftDriveMotorPort,
        SwerveConstants.kRearLeftDriveMotorReversed,
        SwerveConstants.kRearLeftTurningMotorPort,
        SwerveConstants.kRearLeftTurningMotorReversed,
        SwerveConstants.kRearLeftCANcoderPort,
        SwerveConstants.kRearLeftCANcoderDirection,
        SwerveConstants.kRearLeftCANcoderMagnetOffset,
        "RL"
      );
    this.m_rearRightSwerveModule =
      new RobotWheelMover(
        SwerveConstants.kRearRightDriveMotorPort,
        SwerveConstants.kRearRightDriveMotorReversed,
        SwerveConstants.kRearRightTurningMotorPort,
        SwerveConstants.kRearRightTurningMotorReversed,
        SwerveConstants.kRearRightCANcoderPort,
        SwerveConstants.kRearRightCANcoderDirection,
        SwerveConstants.kRearRightCANcoderMagnetOffset,
        "RR"
      );

    this.swerve =
      new SwerveDrive(
        new Config(
          communicator,
          new Wheel[] {
            new Wheel(
              SwerveConstants.rearLeftTranslation,
              m_rearLeftSwerveModule
            ),
            new Wheel(
              SwerveConstants.rearRightTranslation,
              m_rearRightSwerveModule
            ),
            new Wheel(
              SwerveConstants.frontRightTranslation,
              m_frontRightSwerveModule
            ),
            new Wheel(
              SwerveConstants.frontLeftTranslation,
              m_frontLeftSwerveModule
            ),
          }
        )
      );

    this.kinematics =
      new SwerveDriveKinematics(
        SwerveConstants.frontLeftTranslation,
        SwerveConstants.frontRightTranslation,
        SwerveConstants.rearLeftTranslation,
        SwerveConstants.rearRightTranslation
      );

    this.odometry =
      new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(getGlobalGyroAngle()),
        this.getSwerveModulePositions(),
        new Pose2d(0, 0, new Rotation2d())
      );
  }

  public void drive(Vec2 velocity, double rotation, double speed) {
    this.drive(
        velocity,
        rotation,
        speed,
        Math.toRadians(CustomMath.wrapTo180(m_gyro.getYaw()))
      );
  }

  public void drive(
    Vec2 velocity,
    double rotation,
    double speed,
    double gyroAngle
  ) {
    System.out.println(velocity.getX() + " " + velocity.getY() + " " + gyroAngle + " " + rotation + " " + speed + " " + m_frontLeftSwerveModule.m_turnRelativeEncoder.getPosition());
    swerve.drive(velocity, gyroAngle, rotation, speed);
  }

  public void odometryTick() {
    odometry.update(
      Rotation2d.fromDegrees(getGlobalGyroAngle()),
      getSwerveModulePositions()
    );
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeftSwerveModule.getPosition(),
      m_frontRightSwerveModule.getPosition(),
      m_rearLeftSwerveModule.getPosition(),
      m_rearRightSwerveModule.getPosition(),
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeftSwerveModule.getState(),
      m_frontRightSwerveModule.getState(),
      m_rearLeftSwerveModule.getState(),
      m_rearRightSwerveModule.getState(),
    };
  }

  public void resetGyro() {
    gyroOffset = m_gyro.getYaw();
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }

  public void resetOdometryPosition(Pose2d newPose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(getGlobalGyroAngle()),
      this.getSwerveModulePositions(),
      newPose
    );
  }

  private double getGlobalGyroAngle() {
    return m_gyro.getYaw() + gyroOffset;
  }

  @Override
  public byte[] getRawConstructedProtoData() {
    var speeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
    var pose = odometry.getPoseMeters();
    return Odometry
      .newBuilder()
      .setX((float) pose.getX())
      .setY((float) pose.getY())
      .setVx((float) speeds.vxMetersPerSecond) // TODO: fix
      .setVy((float) speeds.vyMetersPerSecond) // TODO: fix
      .setTheta((float) pose.getRotation().getDegrees())
      .setTimestamp(System.currentTimeMillis())
      .build()
      .toByteArray();
  }

  @Override
  public String getPublishTopic() {
    return "robot/odometry";
  }
}
