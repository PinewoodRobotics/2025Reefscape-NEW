package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.RobotWheelMover;
import frc.robot.util.Communicator;
import frc.robot.util.CustomMath;
import frc.robot.util.interfaces.IGyroscopeLike;

import org.pwrup.SwerveDrive;
import org.pwrup.util.Config;
import org.pwrup.util.Vec2;
import org.pwrup.util.Wheel;

/**
 * @nate the only reason this is a subsystem is because I understand that it would be quite useful to instance one command at a time for this.
 */
public class SwerveSubsystem extends SubsystemBase {
  public final RobotWheelMover m_frontLeftSwerveModule;
  private final RobotWheelMover m_frontRightSwerveModule;
  private final RobotWheelMover m_rearLeftSwerveModule;
  private final RobotWheelMover m_rearRightSwerveModule;

  private final SwerveDrive swerve;
  private final IGyroscopeLike m_gyro;
  private double gyroOffset = 0;

  public SwerveSubsystem(IGyroscopeLike gyro, Communicator communicator) {
    this.m_gyro = gyro;
    this.m_frontLeftSwerveModule = new RobotWheelMover(
        SwerveConstants.kFrontLeftDriveMotorPort,
        SwerveConstants.kFrontLeftDriveMotorReversed,
        SwerveConstants.kFrontLeftTurningMotorPort,
        SwerveConstants.kFrontLeftTurningMotorReversed,
        SwerveConstants.kFrontLeftCANcoderPort,
        SwerveConstants.kFrontLeftCANcoderDirection,
        SwerveConstants.kFrontLeftCANcoderMagnetOffset);
    this.m_frontRightSwerveModule = new RobotWheelMover(
        SwerveConstants.kFrontRightDriveMotorPort,
        SwerveConstants.kFrontRightDriveMotorReversed,
        SwerveConstants.kFrontRightTurningMotorPort,
        SwerveConstants.kFrontRightTurningMotorReversed,
        SwerveConstants.kFrontRightCANcoderPort,
        SwerveConstants.kFrontRightCANcoderDirection,
        SwerveConstants.kFrontRightCANcoderMagnetOffset);
    this.m_rearLeftSwerveModule = new RobotWheelMover(
        SwerveConstants.kRearLeftDriveMotorPort,
        SwerveConstants.kRearLeftDriveMotorReversed,
        SwerveConstants.kRearLeftTurningMotorPort,
        SwerveConstants.kRearLeftTurningMotorReversed,
        SwerveConstants.kRearLeftCANcoderPort,
        SwerveConstants.kRearLeftCANcoderDirection,
        SwerveConstants.kRearLeftCANcoderMagnetOffset);
    this.m_rearRightSwerveModule = new RobotWheelMover(
        SwerveConstants.kRearRightDriveMotorPort,
        SwerveConstants.kRearRightDriveMotorReversed,
        SwerveConstants.kRearRightTurningMotorPort,
        SwerveConstants.kRearRightTurningMotorReversed,
        SwerveConstants.kRearRightCANcoderPort,
        SwerveConstants.kRearRightCANcoderDirection,
        SwerveConstants.kRearRightCANcoderMagnetOffset);

    this.swerve = new SwerveDrive(
        new Config(
            communicator,
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
  }

  public void drive(Vec2 velocity, double rotation, double speed) {
    this.drive(
        velocity,
        rotation,
        speed,
        Math.toRadians(CustomMath.wrapTo180(getGlobalGyroAngle())));
  }

  public void drive(
      Vec2 velocity,
      double rotation,
      double speed,
      double gyroAngle) {
    swerve.drive(velocity, gyroAngle, rotation, speed);
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
    gyroOffset = -m_gyro.getYaw();
  }

  private double getGlobalGyroAngle() {
    return m_gyro.getYaw() + gyroOffset;
  }
}
