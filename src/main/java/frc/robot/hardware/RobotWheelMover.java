package frc.robot.hardware;

import org.pwrup.motor.WheelMover;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class RobotWheelMover extends WheelMover {

  private TalonFX m_driveMotor;
  private TalonFX m_turnMotor;
  private SparkClosedLoopController m_turnPIDController;
  public RelativeEncoder m_turnRelativeEncoder;
  public RelativeEncoder m_driveRelativeEncoder;

  private CANcoder turnCANcoder;

  public RobotWheelMover(
      int driveMotorChannel,
      InvertedValue driveMotorReversed,
      int turnMotorChannel,
      InvertedValue turnMotorReversed,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turnMotor = new TalonFX(turnMotorChannel);
    m_turnPIDController = m_turnMotor.getRotorPosition
    m_turnRelativeEncoder = m_turnMotor.getEncoder();

    turnCANcoder = new CANcoder(CANCoderEncoderChannel);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = -CANCoderMagnetOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.SensorDirection = CANCoderDirection;
    turnCANcoder.getConfigurator().apply(config);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration()
    .withMotorOutput(
      new MotorOutputConfigs()
      .withInverted(driveMotorReversed)
      .withNeutralMode(NeutralModeValue.Brake)
    ).withCurrentLimits(
      new CurrentLimitsConfigs()
      .withStatorCurrentLimit(SwerveConstants.kDriveStatorLimit)
      .withSupplyCurrentLimit(SwerveConstants.kDriveSupplyLimit)
    ).withFeedback(
      new FeedbackConfigs()
      .withSensorToMechanismRatio(SwerveConstants.kDriveGearRatio)
    );
    // driveConfig.encoder.velocityConversionFactor(
    //     (Math.PI * SwerveConstants.kWheelDiameterMeters) /
    //         (60 * SwerveConstants.kDriveGearRatio));
    

    TalonFXConfiguration turnConfig = new TalonFXConfiguration()
    .withMotorOutput(
      new MotorOutputConfigs()
      .withInverted(turnMotorReversed)
      .withNeutralMode(NeutralModeValue.Brake)
    ).withCurrentLimits(
      new CurrentLimitsConfigs()
      .withStatorCurrentLimit(SwerveConstants.kTurnStatorLimit)
      .withSupplyCurrentLimit(SwerveConstants.kTurnSupplyLimit)
    ).withFeedback(
      new FeedbackConfigs()
      .withSensorToMechanismRatio(SwerveConstants.kTurnConversionFactor)
    );
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
