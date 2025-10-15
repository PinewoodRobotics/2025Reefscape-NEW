package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.CustomMath;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem self;

  // LEFT MOTOR IS THE LEADER
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private PIDController m_pid;
  private ElevatorFeedforward m_feedforward;

  private Distance m_setpoint = ElevatorConstants.kStartingHeight;
  private Distance m_currentSetpoint = ElevatorConstants.kStartingHeight;

  public static ElevatorSubsystem GetInstance() {
    if (self == null) {
      self = new ElevatorSubsystem();
    }

    return self;
  }

  public ElevatorSubsystem() {
    m_leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

    m_pid = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD);
    m_pid.setTolerance(ElevatorConstants.kTolerance);
    m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV,
        ElevatorConstants.kA);

    configureMotors();
  }

  private void configureMotors() {
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.inverted(ElevatorConstants.kLeftMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30).encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);

    m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight.in(Feet));

    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.inverted(ElevatorConstants.kRightMotorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30).encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);

    m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight.in(Feet));

    m_pid.setIZone(ElevatorConstants.kIZone);
  }

  public void setHeight(Distance height) {
    if (height.gt(ElevatorConstants.kMaxHeight)) {
      System.out.println("WARNING: tried to exceed elevator max height: " + height.in(Feet));
      height = ElevatorConstants.kMaxHeight;
    } else if (height.lt(ElevatorConstants.kMinHeight)) {
      System.out.println("WARNING: tried to exceed elevator min height: " + height.in(Feet));
      height = ElevatorConstants.kMinHeight;
    }
    m_setpoint = height;
  }

  public Distance getAverageHeight() {
    double height = (m_leftMotor.getEncoder().getPosition() + m_rightMotor.getEncoder().getPosition()) / 2.0;
    return Distance.ofRelativeUnits(height, Feet);
  }

  private double calculateSpeed(Distance setpoint) {
    double motorPowerPid = m_pid.calculate(getAverageHeight().in(Feet), setpoint.in(Feet));
    double ff = calculateFeedForwardValue(m_feedforward);
    return MathUtil.clamp(motorPowerPid + ff, -1, 1);
  }

  public boolean atTarget() {
    return Math.abs(getAverageHeight().minus(m_setpoint).in(Feet)) < ElevatorConstants.kTolerance;
  }

  public void stopMotors() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  private double calculateFeedForwardValue(ElevatorFeedforward feedforward) {
    double currentVelocity = m_leftMotor.getEncoder().getVelocity();
    return feedforward.calculate(currentVelocity);
  }

  /**
   * Smoothens the setpoint for not shockloading the algae mechanism on the
   * folding wheels
   * If crossing the resting height, first makes the setpoint the resting height
   * until it can safely keep going
   * 
   * @return the new setpoint
   */
  private Distance smoothRestingHeight(Distance set) {
    if ((set.lt(ElevatorConstants.kRestingHeight)
        && getAverageHeight().minus(ElevatorConstants.kRestingHeight).in(Feet) > ElevatorConstants.kTolerance)
        || (set.gt(ElevatorConstants.kRestingHeight)
            && getAverageHeight().minus(ElevatorConstants.kRestingHeight).in(Feet) < -ElevatorConstants.kTolerance)) {
      return ElevatorConstants.kRestingHeight;
    } else {
      return set;
    }
  }

  private Distance rampSetpoint(Distance set) {
    return Distance.ofRelativeUnits(
        CustomMath.rampSetpoint(set.in(Feet), m_currentSetpoint.in(Feet), ElevatorConstants.kMaxSetpointRamp), Feet);
  }

  private Distance calculateTemporarySetpoint(Distance set) {
    // set = smoothRestingHeight(set);
    set = rampSetpoint(set);
    return set;
  }

  public void resetIAccum() {
    m_leftMotor.getClosedLoopController().setIAccum(0);
    m_rightMotor.getClosedLoopController().setIAccum(0);
  }

  /**
   * 
   * @return The distance between the two motors (left - right)
   */
  /*
   * public Distance getHeightDifference() {
   * return Distance.ofRelativeUnits(m_leftMotor.getEncoder().getPosition() -
   * m_rightMotor.getEncoder().getPosition(), Feet);
   * }
   */

  /**
   * Calculates speeds using the pid controller. Leaves the left motor speed
   * alone, and then
   * adjusts the right motors speed based on how far apart the motors are
   * 
   */
  @Override
  public void periodic() {
    Logger.recordOutput("elevator/setpoint", m_setpoint.in(Feet));
    Logger.recordOutput("elevator/currentSetpoint", m_currentSetpoint.in(Feet));
    Logger.recordOutput("elevator/atTarget", atTarget());
    Logger.recordOutput("elevator/leftMotor", m_leftMotor.getEncoder().getPosition());
    Logger.recordOutput("elevator/rightMotor", m_rightMotor.getEncoder().getPosition());

    m_currentSetpoint = calculateTemporarySetpoint(m_setpoint);

    double speed = calculateSpeed(m_currentSetpoint);

    m_leftMotor.setVoltage(speed * 12);
    m_rightMotor.setVoltage(speed * 12);

    if (m_setpoint != ElevatorConstants.kDefaultHeight) {
      SwerveConstants.tempMaxSpeed = SwerveConstants.kMaxSpeedMPSTopElevator;
    } else {
      SwerveConstants.tempMaxSpeed = SwerveConstants.kMaxSpeedMPSNormElevator;
    }
  }
}
