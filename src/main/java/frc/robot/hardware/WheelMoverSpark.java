package frc.robot.hardware;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.swerve.SwerveConstants;

public class WheelMoverSpark extends WheelMoverBase {

  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;
  private final int driveMotorPort;
  private final SparkClosedLoopController m_drivePIDController, m_turnPIDController;
  private final CANcoder turnCANcoder;
  private final RelativeEncoder m_driveRelativeEncoder, m_rotationRelativeEncoder;
  /**
   * Spark encoder conversion factor for turn position.
   * <p>
   * We command/wrap turn in <b>radians</b>, so the relative encoder must also
   * report <b>radians</b>.
   * <p>
   * {@link SwerveConstants#kTurnConversionFactor} is treated as
   * <b>motor rotations per module rotation</b>, so:
   * 
   * <pre>
   * radiansPerMotorRotation = 2π / (motorRotationsPerModuleRotation)
   * </pre>
   */
  private final double m_turnRadiansPerMotorRotation;

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

    driveMotorPort = driveMotorChannel;
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnMotorChannel, MotorType.kBrushless);

    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turnPIDController = m_turnMotor.getClosedLoopController();

    turnCANcoder = new CANcoder(CANCoderEncoderChannel);
    configureCANCoder(CANCoderDirection, CANCoderMagnetOffset);

    m_turnRadiansPerMotorRotation = c.kTurnConversionFactor;

    configureDriveMotor(driveMotorReversed, c);
    m_driveRelativeEncoder = m_driveMotor.getEncoder();

    configureTurnMotor(turnMotorReversed, c);
    m_rotationRelativeEncoder = m_turnMotor.getEncoder();
    m_rotationRelativeEncoder.setPosition(turnCANcoder.getPosition().getValueAsDouble());
  }

  /** Configures the CANCoder with magnet offset and sensor direction. */
  private void configureCANCoder(SensorDirectionValue direction, double magnetOffset) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.SensorDirection = direction;
    turnCANcoder.getConfigurator().apply(config);
  }

  /** Configures the drive motor with PID, current limit, and encoder settings. */
  private void configureDriveMotor(boolean reversed, SwerveConstants c) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(reversed)
        .smartCurrentLimit(c.kDriveCurrentLimit);

    final double factor = (Math.PI * c.kWheelDiameterMeters) /
        c.kDriveGearRatio;

    config.encoder
        .positionConversionFactor(factor)
        .velocityConversionFactor(factor / 60);

    config.absoluteEncoder
        .positionConversionFactor(factor)
        .velocityConversionFactor(factor / 60);

    config.alternateEncoder
        .positionConversionFactor(factor)
        .velocityConversionFactor(factor / 60);

    /*
     * config.closedLoop
     * .pidf(c.kDriveP, c.kDriveI, c.kDriveD, c.kDriveFF)
     * .iZone(c.kDriveIZ);
     */

    m_driveMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** Configures the turn motor with PID, current limit, and position wrapping. */
  private void configureTurnMotor(boolean reversed, SwerveConstants c) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(reversed)
        .smartCurrentLimit(c.kTurnCurrentLimit);

    config.closedLoop
        .pid(c.kTurnP, c.kTurnI, c.kTurnD)
        .iZone(c.kTurnIZ)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(-0.5)
        .positionWrappingMaxInput(0.5);

    config.encoder.positionConversionFactor(m_turnRadiansPerMotorRotation);
    config.encoder.velocityConversionFactor(m_turnRadiansPerMotorRotation / 60.0);

    m_turnMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /***************************************************************************************************/

  @Override
  public Angle getAngle() {
    // Turn relative encoder is configured to report radians.
    return Angle.ofRelativeUnits(-m_rotationRelativeEncoder.getPosition(), Units.Radians);
  }

  @Override
  public LinearVelocity getSpeed() {
    return LinearVelocity.ofRelativeUnits(m_driveRelativeEncoder.getVelocity(), Units.MetersPerSecond);
  }

  @Override
  public Distance getDistance() {
    return Distance.ofRelativeUnits(m_driveRelativeEncoder.getPosition(), Units.Meters);
  }

  /***************************************************************************************************/

  /** Sets drive speed in meters/sec using SparkMax velocity closed-loop. */
  @Override
  protected void setSpeed(LinearVelocity mpsSpeed) {
    m_drivePIDController.setReference(
        mpsSpeed.in(Units.MetersPerSecond),
        ControlType.kVelocity);
  }

  /** Sets module azimuth in radians using SparkMax position closed-loop. */
  @Override
  protected void turnWheel(Angle newRotationRad) {
    m_turnPIDController.setReference(
        newRotationRad.in(Units.Radians),
        ControlType.kPosition);
  }

  @Override
  public void drive(Angle angle, LinearVelocity speed) {
    setSpeed(speed);
    turnWheel(angle);
    logEverything(speed, angle);
  }

  @Override
  public double getCurrentAngle() {
    return getAngle().in(Units.Radians);
  }

  @Override
  public Rotation2d getRotation2d() {
    // Rotation2d ctor expects radians.
    return new Rotation2d(getAngle().in(Units.Radians));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDistance().in(Units.Meters),
        getRotation2d());
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed().in(Units.MetersPerSecond), getRotation2d());
  }

  @Override
  public void reset() {
    m_rotationRelativeEncoder.setPosition(0);
    m_driveRelativeEncoder.setPosition(0);
  }

  private static boolean invertedValueToSparkInverted(InvertedValue v) {
    // Treat "counterclockwise positive" as "invert output" for SparkMax.
    return v == InvertedValue.CounterClockwise_Positive;
  }

  private void logEverything(LinearVelocity requestedMps, Angle requestedAngle) {
    String base = "Wheels/" + driveMotorPort + "/";
    LinearVelocity actualMps = getSpeed();
    Angle actualAngle = getAngle();
    Distance actualDistance = getDistance();

    // Requested setpoints
    Logger.recordOutput(base + "requested/speedMps", requestedMps.in(Units.MetersPerSecond));
    Logger.recordOutput(base + "requested/angleDeg", requestedAngle.in(Units.Degrees));
    Logger.recordOutput(base + "requested/angleRad", requestedAngle.in(Units.Radians));

    // Actual (encoder-derived) module values
    Logger.recordOutput(base + "actual/speedMps", actualMps.in(Units.MetersPerSecond));
    Logger.recordOutput(base + "actual/distanceM", actualDistance.in(Units.Meters));
    Logger.recordOutput(base + "actual/angleDeg", actualAngle.in(Units.Degrees));
    Logger.recordOutput(base + "actual/angleRad", actualAngle.in(Units.Radians));

    // Drive encoder (configured to meters and m/s)
    Logger.recordOutput(base + "driveEncoder/positionM", m_driveRelativeEncoder.getPosition());
    Logger.recordOutput(base + "driveEncoder/velocityMps", m_driveRelativeEncoder.getVelocity());

    // Turn encoder (configured to radians and rad/s)
    Logger.recordOutput(base + "turnEncoder/positionRad", m_rotationRelativeEncoder.getPosition());
    Logger.recordOutput(base + "turnEncoder/velocityRadPerSec", m_rotationRelativeEncoder.getVelocity());

    // Absolute CANCoder (CTRE reports rotations [0,1) unless configured otherwise)
    double cancoderRot = turnCANcoder.getAbsolutePosition().getValueAsDouble();
    Logger.recordOutput(base + "cancoder/absRotations", cancoderRot);
    Logger.recordOutput(base + "cancoder/absDegrees", cancoderRot * 360.0);
    Logger.recordOutput(base + "cancoder/absRadians", cancoderRot * 2.0 * Math.PI);
  }
}
