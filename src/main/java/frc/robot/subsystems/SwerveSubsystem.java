package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.pwrup.SwerveDrive;
import org.pwrup.util.Config;
import org.pwrup.util.Vec2;
import org.pwrup.util.Wheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.hardware.RobotWheelMover;
import frc.robot.util.CustomMath;
import pwrup.frc.core.hardware.sensor.IGyroscopeLike;

/**
 * @nate the only reason this is a subsystem is because I understand that it
 *       would be quite useful to instance one command at a time for this.
 */
public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem self;
  public final RobotWheelMover m_frontLeftSwerveModule;
  private final RobotWheelMover m_frontRightSwerveModule;
  private final RobotWheelMover m_rearLeftSwerveModule;
  private final RobotWheelMover m_rearRightSwerveModule;

  private final SwerveDrive swerve;
  private final IGyroscopeLike m_gyro;
  private double gyroOffset = 0;
  private boolean shouldWork = true;

  private final SwerveDriveKinematics kinematics;

  public static SwerveSubsystem GetInstance() {
    if (self == null) {
      self = new SwerveSubsystem(AHRSGyro.GetInstance());
    }

    return self;
  }

  public SwerveSubsystem(IGyroscopeLike gyro) {
    this.m_gyro = gyro;
    this.m_frontLeftSwerveModule = new RobotWheelMover(
        SwerveConstants.kFrontLeftDriveMotorPort,
        SwerveConstants.kFrontLeftDriveMotorReversed,
        SwerveConstants.kFrontLeftTurningMotorPort,
        SwerveConstants.kFrontLeftTurningMotorReversed,
        SwerveConstants.kFrontLeftCANcoderPort,
        SwerveConstants.kFrontLeftCANcoderDirection,
        SwerveConstants.kFrontLeftCANcoderMagnetOffset,
        SwerveConstants.tempMaxSpeed);
    this.m_frontRightSwerveModule = new RobotWheelMover(
        SwerveConstants.kFrontRightDriveMotorPort,
        SwerveConstants.kFrontRightDriveMotorReversed,
        SwerveConstants.kFrontRightTurningMotorPort,
        SwerveConstants.kFrontRightTurningMotorReversed,
        SwerveConstants.kFrontRightCANcoderPort,
        SwerveConstants.kFrontRightCANcoderDirection,
        SwerveConstants.kFrontRightCANcoderMagnetOffset,
        SwerveConstants.tempMaxSpeed);
    this.m_rearLeftSwerveModule = new RobotWheelMover(
        SwerveConstants.kRearLeftDriveMotorPort,
        SwerveConstants.kRearLeftDriveMotorReversed,
        SwerveConstants.kRearLeftTurningMotorPort,
        SwerveConstants.kRearLeftTurningMotorReversed,
        SwerveConstants.kRearLeftCANcoderPort,
        SwerveConstants.kRearLeftCANcoderDirection,
        SwerveConstants.kRearLeftCANcoderMagnetOffset,
        SwerveConstants.tempMaxSpeed);
    this.m_rearRightSwerveModule = new RobotWheelMover(
        SwerveConstants.kRearRightDriveMotorPort,
        SwerveConstants.kRearRightDriveMotorReversed,
        SwerveConstants.kRearRightTurningMotorPort,
        SwerveConstants.kRearRightTurningMotorReversed,
        SwerveConstants.kRearRightCANcoderPort,
        SwerveConstants.kRearRightCANcoderDirection,
        SwerveConstants.kRearRightCANcoderMagnetOffset,
        SwerveConstants.tempMaxSpeed);

    this.swerve = new SwerveDrive(
        new Config(
            Optional.empty(),
            new Wheel[] {
                new Wheel(
                    SwerveConstants.frontRightTranslation,
                    m_frontRightSwerveModule),
                new Wheel(
                    SwerveConstants.frontLeftTranslation,
                    m_frontLeftSwerveModule),
                new Wheel(
                    SwerveConstants.rearLeftTranslation,
                    m_rearLeftSwerveModule),
                new Wheel(
                    SwerveConstants.rearRightTranslation,
                    m_rearRightSwerveModule),
            }));

    this.kinematics = new SwerveDriveKinematics(
        SwerveConstants.frontLeftTranslation,
        SwerveConstants.frontRightTranslation,
        SwerveConstants.rearLeftTranslation,
        SwerveConstants.rearRightTranslation);
  }

  public void stop() {
    driveRaw(new ChassisSpeeds(0, 0, 0));
  }

  public enum DriveType {
    GYRO_RELATIVE,
    RAW,
  }

  public void drive(ChassisSpeeds speeds, DriveType driveType) {
    if (!shouldWork) {
      stop();
      return;
    }

    switch (driveType) {
      case GYRO_RELATIVE:
        driveFieldRelative(speeds);
        break;
      case RAW:
        driveRaw(speeds);
        break;
    }
  }

  public void driveRaw(ChassisSpeeds speeds) {
    var actualSpeeds = toSwerveOrientation(speeds);
    swerve.driveNonRelative(actualSpeeds);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    var actualSpeeds = toSwerveOrientation(speeds);
    swerve.driveWithGyro(actualSpeeds, new Rotation2d(getSwerveGyroAngle()));
  }

  public static ChassisSpeeds fromPercentToVelocity(Vec2 percentXY, double rotationPercent) {
    double vx = clamp(percentXY.getX(), -1, 1) * SwerveConstants.tempMaxSpeed;
    double vy = clamp(percentXY.getY(), -1, 1) * SwerveConstants.tempMaxSpeed;
    double omega = clamp(rotationPercent, -1, 1) * SwerveConstants.kMaxAngularSpeedRadPerSec;
    return new ChassisSpeeds(vx, vy, omega);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeftSwerveModule.getPosition(),
        m_frontRightSwerveModule.getPosition(),
        m_rearLeftSwerveModule.getPosition(),
        m_rearRightSwerveModule.getPosition(),
    };
  }

  public ChassisSpeeds getGlobalChassisSpeeds(Rotation2d heading) {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), heading);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
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
    resetGyro(0);
  }

  public void resetGyro(double offset) {
    gyroOffset = -m_gyro.getYaw() + offset;
  }

  public double getSwerveGyroAngle() {
    return Math.toRadians(CustomMath.wrapTo180(m_gyro.getYaw() + gyroOffset));
  }

  public void setShouldWork(boolean value) {
    this.shouldWork = value;
    if (!shouldWork) {
      stop(); // make sure it applies immediately
    }
  }

  private static double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(max, v));
  }

  private static ChassisSpeeds toSwerveOrientation(ChassisSpeeds target) {
    return new ChassisSpeeds(
        -target.vxMetersPerSecond,
        target.vyMetersPerSecond,
        target.omegaRadiansPerSecond);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SwerveSubsystem/swerve/states", getSwerveModuleStates());
  }
}
