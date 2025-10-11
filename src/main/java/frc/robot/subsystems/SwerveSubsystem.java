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
  private boolean masterDriveRawSwitch = false;

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
        SwerveConstants.kMaxSpeedMPS);
    this.m_frontRightSwerveModule = new RobotWheelMover(
        SwerveConstants.kFrontRightDriveMotorPort,
        SwerveConstants.kFrontRightDriveMotorReversed,
        SwerveConstants.kFrontRightTurningMotorPort,
        SwerveConstants.kFrontRightTurningMotorReversed,
        SwerveConstants.kFrontRightCANcoderPort,
        SwerveConstants.kFrontRightCANcoderDirection,
        SwerveConstants.kFrontRightCANcoderMagnetOffset,
        SwerveConstants.kMaxSpeedMPS);
    this.m_rearLeftSwerveModule = new RobotWheelMover(
        SwerveConstants.kRearLeftDriveMotorPort,
        SwerveConstants.kRearLeftDriveMotorReversed,
        SwerveConstants.kRearLeftTurningMotorPort,
        SwerveConstants.kRearLeftTurningMotorReversed,
        SwerveConstants.kRearLeftCANcoderPort,
        SwerveConstants.kRearLeftCANcoderDirection,
        SwerveConstants.kRearLeftCANcoderMagnetOffset,
        SwerveConstants.kMaxSpeedMPS);
    this.m_rearRightSwerveModule = new RobotWheelMover(
        SwerveConstants.kRearRightDriveMotorPort,
        SwerveConstants.kRearRightDriveMotorReversed,
        SwerveConstants.kRearRightTurningMotorPort,
        SwerveConstants.kRearRightTurningMotorReversed,
        SwerveConstants.kRearRightCANcoderPort,
        SwerveConstants.kRearRightCANcoderDirection,
        SwerveConstants.kRearRightCANcoderMagnetOffset,
        SwerveConstants.kMaxSpeedMPS);

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

  /**
   * 
   * @param velocity mps!
   * @param rotation rotation in % (-1, 1)
   */
  public void driveVel(Vec2 velocity, double rotation) {
    swerve.drive(velocity, rotation, 1);
  }

  public void drive(Vec2 velocity, double rotation) {
    this.drive(
        velocity,
        rotation,
        1,
        Math.toRadians(CustomMath.wrapTo180(getGlobalGyroAngle())));
  }

  public void drive(Vec2 velocity, double rotation, double speed) {
    this.drive(
        velocity.scaleToModulo(speed),
        rotation,
        1,
        Math.toRadians(CustomMath.wrapTo180(getGlobalGyroAngle())));
  }

  public void drive(
      Vec2 velocity,
      double rotation,
      double speed,
      double gyroAngle) {
    swerve.drive(velocity, gyroAngle, rotation, speed);
  }

  public void driveRaw(Vec2 velocity, double rotation, double speed) {
    if (!masterDriveRawSwitch) {
      swerve.drive(velocity, rotation, speed);
    } else {
      swerve.drive(new Vec2(0, 0), 0, 0);
    }
  }

  public static Vec2 toSwerveOrientation(Translation2d target) {
    return new Vec2(-target.getX(), target.getY()).scaleToModulo(1);
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
    gyroOffset = -m_gyro.getYaw();
  }

  public void resetGyro(double offset) {
    gyroOffset = -m_gyro.getYaw() + offset;
  }

  public double getGlobalGyroAngle() {
    return CustomMath.wrapTo180(m_gyro.getYaw() + gyroOffset);
  }

  private void masterDriveRawSwitch(boolean value) {
    this.masterDriveRawSwitch = value;
    if (value) {
      driveRaw(null, 0, 0); // make sure it applies immediately
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SwerveSubsystem/swerve/states", getSwerveModuleStates());
  }
}
