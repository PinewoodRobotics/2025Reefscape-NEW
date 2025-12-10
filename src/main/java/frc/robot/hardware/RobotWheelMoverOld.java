package frc.robot.hardware;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.pwrup.motor.WheelMover;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.SwerveConstants;

public class RobotWheelMoverOld extends WheelMover {

  private TalonFX m_driveMotor;
  private TalonFX m_turnMotor;
  private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
  private final int port;

  private CANcoder turnCANcoder;

  public RobotWheelMoverOld(
      int driveMotorChannel,
      InvertedValue driveMotorReversed,
      int turnMotorChannel,
      InvertedValue turnMotorReversed,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    this.port = driveMotorChannel;
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turnMotor = new TalonFX(turnMotorChannel);

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
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(
                    SwerveConstants.kDriveStatorLimit)
                .withSupplyCurrentLimit(
                    SwerveConstants.kDriveSupplyLimit))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    SwerveConstants.kDriveGearRatio))
        .withSlot0(
            new Slot0Configs()
                .withKP(SwerveConstants.kDriveP)
                .withKI(SwerveConstants.kDriveI)
                .withKD(SwerveConstants.kDriveD)
                .withKV(SwerveConstants.kDriveV))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(SwerveConstants.kDriveMotionMagicAcceleration)
                .withMotionMagicJerk(SwerveConstants.kDriveMotionMagicJerk));

    m_driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(turnMotorReversed)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(
                    SwerveConstants.kTurnStatorLimit)
                .withSupplyCurrentLimit(
                    SwerveConstants.kTurnSupplyLimit))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(
                    SwerveConstants.kTurnConversionFactor))
        .withSlot0(
            new Slot0Configs()
                .withKP(SwerveConstants.kTurnP)
                .withKI(SwerveConstants.kTurnI)
                .withKD(SwerveConstants.kTurnD))
        .withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs().withContinuousWrap(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(SwerveConstants.kTurnMotionMagicCruiseVelocity)
                .withMotionMagicAcceleration(SwerveConstants.kTurnMotionMagicAcceleration)
                .withMotionMagicJerk(SwerveConstants.kTurnMotionMagicJerk));

    m_turnMotor.getConfigurator().apply(turnConfig);
    m_turnMotor.setPosition(
        turnCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public void setSpeed(double mpsSpeed) {
    double wheelCircumference = Math.PI * SwerveConstants.kWheelDiameterMeters;
    double wheelRps = mpsSpeed / wheelCircumference;

    Logger.recordOutput("Wheels/" + port + "/requestedMps", mpsSpeed);
    Logger.recordOutput("Wheels/" + port + "/wheelRps", wheelRps);
    Logger.recordOutput("Wheels/" + port + "/actualMps", getState().speedMetersPerSecond);

    m_driveMotor.setControl(velocityRequest.withVelocity(wheelRps));
  }

  public void turnWheel(Angle newRotation) {
    m_turnMotor.setControl(
        positionRequest.withPosition(newRotation));
  }

  @Override
  public void drive(double angle, double speed) {
    Logger.recordOutput("AAA", speed);
    setSpeed(speed);
    turnWheel(Angle.ofRelativeUnits(angle, Radians));

    Logger.recordOutput("Wheels/" + port + "/mps", getState().speedMetersPerSecond);
  }

  @Override
  public double getCurrentAngle() {
    return fromRotationsToRadians(m_turnMotor.getPosition().getValueAsDouble());
  }

  private double fromRotationsToRadians(double rotations) {
    return rotations * 2 * Math.PI;
  }

  /**
   * Converts wheel rotations to distance/velocity in meters
   * Note: With SensorToMechanismRatio configured, motor values are already in
   * wheel rotations
   * 
   * @param wheelRotations Wheel position or velocity in rotations
   * @return Wheel distance/velocity in meters
   */
  private double convertWheelRotationsToMeters(double wheelRotations) {
    return -wheelRotations * (Math.PI * SwerveConstants.kWheelDiameterMeters);
  }

  public double getCANCoderAngle() {
    return turnCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(-fromRotationsToRadians(m_turnMotor.getPosition().getValueAsDouble()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        convertWheelRotationsToMeters(m_driveMotor.getPosition().getValueAsDouble()),
        getRotation2d());
  }

  public SwerveModuleState getState() {
    Logger.recordOutput("Wheels/" + port + "/value", m_driveMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(
        convertWheelRotationsToMeters(m_driveMotor.getVelocity().getValueAsDouble()),
        getRotation2d());
  }

  public void reset() {
    m_turnMotor.setPosition(0);
    m_driveMotor.setPosition(0);
  }
}
