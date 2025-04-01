package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    
    private SparkMax m_leftMotor = new SparkMax(AlgaeConstants.leftMotorID, MotorType.kBrushless);
    private SparkMax m_rightMotor = new SparkMax(AlgaeConstants.rightMotorID, MotorType.kBrushless);
    private SparkMax m_wristMotor = new SparkMax(AlgaeConstants.wristMotorID, MotorType.kBrushless);

    private boolean m_hasAlgae = false;

    private Rotation2d m_wristSetpoint = AlgaeConstants.kDefaultAngle;
    
    public AlgaeSubsystem() {
        configureMotors();
        configureWrist();
    }

    private void configureWrist() {
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.smartCurrentLimit(AlgaeConstants.kWristCurrentLimit);
        wristConfig.inverted(AlgaeConstants.kInverted);
        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristConfig.encoder.positionConversionFactor(AlgaeConstants.kGearingRatio);
        wristConfig.idleMode(IdleMode.kBrake);
        wristConfig.closedLoop.pid(
            AlgaeConstants.kP,
            AlgaeConstants.kI,
            AlgaeConstants.kD
            )
        .iZone(AlgaeConstants.kIZone);
        
        wristConfig.absoluteEncoder.inverted(true).zeroOffset(AlgaeConstants.kWristOffset.getRotations());
        m_wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        calibrateWrist();
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

    public void calibrateWrist() {
        m_wristMotor.getEncoder().setPosition(MathFunc.plusMinusHalf(m_wristMotor.getAbsoluteEncoder().getPosition()));
    }

    @Override
    public void periodic() {
        m_wristMotor.getClosedLoopController().setReference(
            m_wristSetpoint.getRotations(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            calculateFeedForward()
        );

    }

    private double calculateFeedForward() {
        return AlgaeConstants.kFF * Math.cos(getWristPosition().getRadians() - AlgaeConstants.kFFOffset.getRadians());
    }

    public void stopWrist() {
        setWristPosition(getWristPosition());
    }

    public void setWristPosition(Rotation2d position) {
        m_wristSetpoint = position;
        System.out.println("moving the wrist!");
    }

     public Rotation2d getWristPosition() {
        return Rotation2d.fromRotations(m_wristMotor.getEncoder().getPosition());
    }

    public Rotation2d getSetpoint() {
        return m_wristSetpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(getWristPosition().minus(m_wristSetpoint).getRotations()) < AlgaeConstants.kTolerance.getRotations();
    }

}
