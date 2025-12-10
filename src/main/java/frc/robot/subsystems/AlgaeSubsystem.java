package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;
import frc.robot.util.CustomMath;

public class AlgaeSubsystem extends SubsystemBase {

    private static AlgaeSubsystem self;

    private SparkMax m_leftMotor = new SparkMax(AlgaeConstants.leftMotorID, MotorType.kBrushless);
    private SparkMax m_rightMotor = new SparkMax(AlgaeConstants.rightMotorID, MotorType.kBrushless);
    private SparkMax m_wrist = new SparkMax(AlgaeConstants.wristMotorID, MotorType.kBrushless);

    private Rotation2d m_wristSetpoint = AlgaeConstants.kWristDefaultAngle;

    private boolean m_hasAlgae = false;

    public static AlgaeSubsystem GetInstance() {
        if (self == null) {
            self = new AlgaeSubsystem();
        }

        return self;
    }

    public AlgaeSubsystem() {
        configureMotors();
        configureWrist();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("algae/wrist/setpoint", m_wristSetpoint.getRotations());
        Logger.recordOutput("algae/wrist/position", getWristPosition().getRotations());
        Logger.recordOutput("algae/wrist/atSetpoint", wristAtSetpoint());
        Logger.recordOutput("algae/wrist/hasAlgae", m_hasAlgae);
        Logger.recordOutput("algae/wrist/leftMotor", m_leftMotor.getEncoder().getPosition());
        Logger.recordOutput("algae/wrist/rightMotor", m_rightMotor.getEncoder().getPosition());

        m_wrist.getClosedLoopController().setReference(
                m_wristSetpoint.getRotations(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                calculateFeedForward());
    }

    private void configureMotors() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(AlgaeConstants.kLeftMotorInverted);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.smartCurrentLimit(AlgaeConstants.kCurrentLimit);
        m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(AlgaeConstants.kRightMotorInverted);
        rightMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.smartCurrentLimit(AlgaeConstants.kCurrentLimit);
        m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureWrist() {
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.smartCurrentLimit(AlgaeConstants.kWristCurrentLimit);
        wristConfig.inverted(AlgaeConstants.kAbsoluteEncoderInverted);
        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristConfig.encoder.positionConversionFactor(AlgaeConstants.kGearingRatio);
        wristConfig.idleMode(IdleMode.kBrake);
        wristConfig.closedLoop.pid(
                AlgaeConstants.kP,
                AlgaeConstants.kI,
                AlgaeConstants.kD)
                .iZone(AlgaeConstants.kIZone);

        wristConfig.absoluteEncoder.inverted(true).zeroOffset(AlgaeConstants.kWristOffset.getRotations());
        m_wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        calibrateWrist();
    }

    public void calibrateWrist() {
        m_wrist.getEncoder().setPosition(CustomMath.plusMinusHalf(m_wrist.getAbsoluteEncoder().getPosition()));
    }

    public void runMotors() {
        runMotors(AlgaeConstants.kIntakeSpeed);
    }

    /**
     * 
     * @param speed -1 to 1
     */
    public void runMotors(double speed) {
        m_leftMotor.set(speed);
        m_rightMotor.set(speed);
    }

    public void stopMotors() {
        m_leftMotor.set(0);
        m_rightMotor.set(0);
    }

    public void setHoldingAlgae(boolean hasAlgae) {
        m_hasAlgae = hasAlgae;
    }

    public boolean hasAlgae() {
        return m_hasAlgae;
    }

    public void setWristPosition(Rotation2d position) {
        m_wristSetpoint = position;
        System.out.println("moving the wrist!");
    }

    public Rotation2d getWristPosition() {
        return Rotation2d.fromRotations(m_wrist.getEncoder().getPosition());
    }

    private double calculateFeedForward() {
        return AlgaeConstants.kFF * Math.cos(getWristPosition().getRadians());
    }

    public boolean wristAtSetpoint() {
        return Math.abs(m_wristSetpoint.minus(getWristPosition()).getRotations()) < AlgaeConstants.kTolerance;
    }

}
