package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralConstants;
import frc.robot.util.MathFunc;

public class CoralSubsystem extends SubsystemBase {
    private SparkMax m_wrist = new SparkMax(CoralConstants.wristMotorID, MotorType.kBrushless);
    private SparkMax m_intake = new SparkMax(CoralConstants.intakeMotorID, MotorType.kBrushless);

    private boolean m_hasCoral = true;
    

    private Rotation2d m_wristSetpoint = CoralConstants.kDefaultAngle;

    public CoralSubsystem() {
        configureWrist();
        configureIntake();
    }

    private void configureIntake() {
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig.inverted(CoralConstants.kIntakeInverted);
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.smartCurrentLimit(CoralConstants.kIntakeCurrentLimit);

        m_intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureWrist() {
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.smartCurrentLimit(CoralConstants.kWristCurrentLimit);
        wristConfig.inverted(CoralConstants.kInverted);
        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristConfig.encoder.positionConversionFactor(CoralConstants.kGearingRatio);
        wristConfig.idleMode(IdleMode.kBrake);
        wristConfig.closedLoop.pid(
            CoralConstants.kP,
            CoralConstants.kI,
            CoralConstants.kD
            )
        .iZone(CoralConstants.kIZone);
        
        wristConfig.absoluteEncoder.inverted(true).zeroOffset(CoralConstants.kWristOffset.getRotations());
        
        m_wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        calibrateWrist();
    }

    @Override
    public void periodic() {
        m_wrist.getClosedLoopController().setReference(
            m_wristSetpoint.getRotations(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            calculateFeedForward()
        );

    }

    public void intake() {
        runIntake(CoralConstants.kIntakeSpeed);
    }

    public void eject() {
        runIntake(CoralConstants.kEjectSpeed);
    }

    public void runIntake(double speed) {
        m_intake.set(speed);
    }

    private double calculateFeedForward() {
        return CoralConstants.kFF * Math.cos(getWristPosition().getRadians() - CoralConstants.kFFOffset.getRadians());
    }

    public void stopIntake() {
        m_intake.set(0);
    }

    public void stopWrist() {
        setWristPosition(getWristPosition());
    }

    public void setWristPosition(Rotation2d position) {
        m_wristSetpoint = position;
        System.out.println("moving the wrist!");
    }

    public Rotation2d getWristPosition() {
        return Rotation2d.fromRotations(m_wrist.getEncoder().getPosition());
    }

    public Rotation2d getSetpoint() {
        return m_wristSetpoint;
    }

    public void setHoldingCoral(boolean in) {
        m_hasCoral = in;
    }

    public boolean hasCoral() {
        return m_hasCoral;
    }

    public boolean atSetpoint() {
        return Math.abs(getWristPosition().minus(m_wristSetpoint).getRotations()) < CoralConstants.kTolerance.getRotations();
    }

    public double getIntakeCurrent() {
        return m_intake.getOutputCurrent();
    }

    public void calibrateWrist() {
        m_wrist.getEncoder().setPosition(MathFunc.plusMinusHalf(m_wrist.getAbsoluteEncoder().getPosition()));
    }
    
}
