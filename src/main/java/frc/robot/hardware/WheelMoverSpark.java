package frc.robot.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.swerve.SwerveConstants;

public class WheelMoverSpark extends WheelMoverBase {

  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;
  private final SparkClosedLoopController m_drivePIDController;
  private final SparkClosedLoopController m_turnPIDController;
  private final CANcoder turnCANcoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final RelativeEncoder m_driveRelativeEncoder;

  /**
   * Compatibility constructor: allows using the same constants as the
   * TalonFX-based modules.
   */
  public WheelMoverSpark(
      int driveMotorChannel,
      InvertedValue driveMotorInverted,
      int turnMotorChannel,
      InvertedValue turnMotorInverted,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    this(
        driveMotorChannel,
        invertedValueToSparkInverted(driveMotorInverted),
        turnMotorChannel,
        invertedValueToSparkInverted(turnMotorInverted),
        CANCoderEncoderChannel,
        CANCoderDirection,
        CANCoderMagnetOffset);
  }

  public WheelMoverSpark(
      int driveMotorChannel,
      boolean driveMotorReversed,
      int turnMotorChannel,
      boolean turnMotorReversed,
      int CANCoderEncoderChannel,
      SensorDirectionValue CANCoderDirection,
      double CANCoderMagnetOffset) {
    final var c = SwerveConstants.INSTANCE;
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnMotorChannel, MotorType.kBrushless);
    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turnPIDController = m_turnMotor.getClosedLoopController();
    m_turnRelativeEncoder = m_turnMotor.getEncoder();

    turnCANcoder = new CANcoder(CANCoderEncoderChannel);
    CANcoderConfiguration turnCANcoderConfig = new CANcoderConfiguration();
    turnCANcoderConfig.MagnetSensor.MagnetOffset = -CANCoderMagnetOffset;
    turnCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    turnCANcoderConfig.MagnetSensor.SensorDirection = CANCoderDirection;
    turnCANcoder.getConfigurator().apply(turnCANcoderConfig);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(driveMotorReversed)
        .smartCurrentLimit(c.kDriveCurrentLimit);
    // Set encoder units to meters (position) and meters/sec (velocity).
    // Note: we negate here to match RobotWheelMoverNew's sign convention.
    final double wheelCircumference = Math.PI * c.kWheelDiameterMeters;
    final double metersPerMotorRotation = wheelCircumference / c.kDriveGearRatio;
    driveConfig.encoder
        .positionConversionFactor(-metersPerMotorRotation)
        .velocityConversionFactor(-metersPerMotorRotation / 60.0);
    driveConfig.closedLoop
        .pidf(
            c.kDriveP,
            c.kDriveI,
            c.kDriveD,
            c.kDriveFF)
        .iZone(c.kDriveIZ)
        .outputRange(c.kDriveMinOutput, c.kDriveMaxOutput);
    m_driveMotor.configure(
        driveConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_driveRelativeEncoder = m_driveMotor.getEncoder();

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnMotorReversed)
        .smartCurrentLimit(c.kTurnCurrentLimit);
    turnConfig.closedLoop
        .pid(
            c.kTurnP,
            c.kTurnI,
            c.kTurnD)
        .iZone(c.kTurnIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(-0.5)
        .positionWrappingMaxInput(0.5);
    turnConfig.encoder.positionConversionFactor(
        c.kTurnConversionFactor);
    m_turnMotor.configure(
        turnConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turnRelativeEncoder.setPosition(
        turnCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  /** Sets drive speed in meters/sec using SparkMax velocity closed-loop. */
  public void setSpeed(double mpsSpeed) {
    m_drivePIDController.setReference(mpsSpeed, ControlType.kVelocity);
  }

  /** Sets module azimuth in radians using SparkMax position closed-loop. */
  public void turnWheel(Angle newRotation) {
    m_turnPIDController.setReference(
        newRotation.in(edu.wpi.first.units.Units.Radians) / (2 * Math.PI),
        ControlType.kPosition);
  }

  @Override
  public void drive(double angle, double speed) {
    setSpeed(speed);
    turnWheel(Angle.ofRelativeUnits(angle, edu.wpi.first.units.Units.Radians));
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

  @Override
  public Rotation2d getRotation2d() {
    return new Rotation2d(-m_turnRelativeEncoder.getPosition() * 2.0 * Math.PI);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveRelativeEncoder.getPosition(),
        getRotation2d());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveRelativeEncoder.getVelocity(), getRotation2d());
  }

  public void reset() {
    m_turnRelativeEncoder.setPosition(0);
    m_driveRelativeEncoder.setPosition(0);
  }

  private static boolean invertedValueToSparkInverted(InvertedValue v) {
    // Treat "counterclockwise positive" as "invert output" for SparkMax.
    return v == InvertedValue.CounterClockwise_Positive;
  }
}
