package frc.robot.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import org.pwrup.motor.WheelMover;

public class RobotWheelMover extends WheelMover {

  private SparkMax m_driveMotor;
  private SparkMax m_turnMotor;
  private SparkClosedLoopController m_turnPIDController;
  public RelativeEncoder m_turnRelativeEncoder;
  public RelativeEncoder m_driveRelativeEncoder;

  private CANcoder turnCANcoder;

  public RobotWheelMover(
      int driveMotorChannel,
      boolean driveMotorReversed,
      int turnMotorChannel,
      boolean turnMotorReversed,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnMotorChannel, MotorType.kBrushless);
    m_turnPIDController = m_turnMotor.getClosedLoopController();
    m_turnRelativeEncoder = m_turnMotor.getEncoder();

    turnCANcoder = new CANcoder(CANCoderEncoderChannel);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = -CANCoderMagnetOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.SensorDirection = CANCoderDirection;
    turnCANcoder.getConfigurator().apply(config);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(driveMotorReversed)
        .smartCurrentLimit(SwerveConstants.kDriveCurrentLimit);
    driveConfig.encoder.velocityConversionFactor(
        (Math.PI * SwerveConstants.kWheelDiameterMeters) /
            (60 * SwerveConstants.kDriveGearRatio));
    m_driveMotor.configure(
        driveConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_driveRelativeEncoder = m_driveMotor.getEncoder();

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnMotorReversed)
        .smartCurrentLimit(SwerveConstants.kTurnCurrentLimit);
    turnConfig.closedLoop
        .pid(
            SwerveConstants.kTurnP,
            SwerveConstants.kTurnI,
            SwerveConstants.kTurnD)
        .iZone(SwerveConstants.kTurnIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(-0.5)
        .positionWrappingMaxInput(0.5);
    turnConfig.encoder.positionConversionFactor(
        SwerveConstants.kTurnConversionFactor);
    m_turnMotor.configure(
        turnConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turnRelativeEncoder.setPosition(
        turnCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void drive(double angle, double speed) {
    m_driveMotor.set(speed);
    m_turnPIDController.setReference(
        angle / (2 * Math.PI),
        ControlType.kPosition);
  }

  @Override
  public double getCurrentAngle() {
    return fromRotationsToRadians(this.m_turnRelativeEncoder.getPosition());
  }

  private double fromRotationsToRadians(double rotations) {
    return rotations * 2 * Math.PI;
  }

  public double getCANCoderAngle() {
    return turnCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        (m_driveRelativeEncoder.getPosition() / SwerveConstants.kDriveGearRatio) *
            (Math.PI * SwerveConstants.kWheelDiameterMeters),
        new Rotation2d(m_turnRelativeEncoder.getPosition() * 2 * Math.PI));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        (m_driveRelativeEncoder.getVelocity() / SwerveConstants.kDriveGearRatio) *
            (Math.PI * SwerveConstants.kWheelDiameterMeters),
        new Rotation2d(m_turnRelativeEncoder.getPosition() * 2 * Math.PI));
  }

  public void reset() {
    m_turnRelativeEncoder.setPosition(0);
    m_driveRelativeEncoder.setPosition(0);
  }
}
