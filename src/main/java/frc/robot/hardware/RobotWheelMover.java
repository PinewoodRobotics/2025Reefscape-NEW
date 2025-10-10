package frc.robot.hardware;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.pwrup.motor.WheelMover;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class RobotWheelMover extends WheelMover {

  private TalonFX m_driveMotor;
  private TalonFX m_turnMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final double maxSpeedMPS;
  private final int port;

  private CANcoder turnCANcoder;

  public RobotWheelMover(
      int driveMotorChannel,
      InvertedValue driveMotorReversed,
      int turnMotorChannel,
      InvertedValue turnMotorReversed,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset,
      double maxSpeedMPS) {
    this.port = driveMotorChannel;
    this.maxSpeedMPS = maxSpeedMPS;
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
                .withKV(SwerveConstants.kDriveV));

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
            new ClosedLoopGeneralConfigs().withContinuousWrap(true));

    m_turnMotor.getConfigurator().apply(turnConfig);
    m_turnMotor.setPosition(
        turnCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public void setSpeed(double mpsSpeed) {
    double wheelCircumference = Math.PI * SwerveConstants.kWheelDiameterMeters;
    double wheelRps = mpsSpeed / wheelCircumference;

    Logger.recordOutput("Wheels/" + port + "/rps", wheelRps);

    m_driveMotor.setControl(velocityRequest.withVelocity(wheelRps));
  }

  public void turnWheel(Angle newRotation) {
    m_turnMotor.setControl(
        new PositionVoltage(newRotation));
  }

  @Override
  public void drive(double angle, double speed) {
    setSpeed(maxSpeedMPS * speed);
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
   * Converts raw motor value to wheel distance/velocity in meters
   * 
   * @param motorValue Raw motor position or velocity value
   * @return Wheel distance/velocity in meters
   */
  private double convertMotorToWheelDistance(double motorValue) {
    return (-motorValue /
        (SwerveConstants.kDriveGearRatio *
            SwerveConstants.kThursdayHackGearRatio))
        *
        (Math.PI * SwerveConstants.kWheelDiameterMeters);
  }

  public double getCANCoderAngle() {
    return turnCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(-m_turnMotor.getPosition().getValueAsDouble() * 2 * Math.PI);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        convertMotorToWheelDistance(m_driveMotor.getPosition().getValueAsDouble()),
        getRotation2d());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        convertMotorToWheelDistance(m_driveMotor.getVelocity().getValueAsDouble()),
        getRotation2d());
  }

  public void reset() {
    m_turnMotor.setPosition(0);
    m_driveMotor.setPosition(0);
  }
}
