package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.MathFunc;

public class ElevatorSubsystem extends SubsystemBase {
    
    //LEFT MOTOR IS THE LEADER
    private SparkMax m_leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
    private SparkMax m_rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
    private PIDController m_pid = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD
    );

    private double m_setpoint = ElevatorConstants.kStartingHeight;
    private double m_currentSetpoint = ElevatorConstants.kStartingHeight;

    public ElevatorSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(ElevatorConstants.kLeftMotorInverted)
            .idleMode(IdleMode.kCoast)
            .encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);
            
        m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight);
        

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(ElevatorConstants.kRightMotorInverted)
            .idleMode(IdleMode.kCoast)
            .encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);
        m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight);

        m_pid.setIZone(ElevatorConstants.kIZone);
        
    }

    public void setHeight(Distance height) {
        m_setpoint = height.in(Feet);
    }

    public Distance getAverageHeight() {
        double height = (m_leftMotor.getEncoder().getPosition() + m_rightMotor.getEncoder().getPosition()) / 2.0;
        return Distance.ofRelativeUnits(height, Feet);
    }


    /**
     * 
     * @return The distance between the two motors (left - right)
     */
    public Distance getHeightDifference() {
        return Distance.ofRelativeUnits(m_leftMotor.getEncoder().getPosition() - m_rightMotor.getEncoder().getPosition(), Feet);
    }

    
    /**
     *  Calculates speeds using the pid controller. Leaves the left motor speed alone, and then
     * adjusts the right motors speed based on how far apart the motors are
     * 
     */    
    @Override
    public void periodic() {
        // m_currentSetpoint = MathFunc.rampSetpoint(m_setpoint, m_currentSetpoint, ElevatorConstants.kMaxSetpointRamp);
        m_currentSetpoint = m_setpoint;
        double totalSpeed = m_pid.calculate(getAverageHeight().in(Feet), m_currentSetpoint);
        
        double rightMotorSpeed = totalSpeed + getHeightDifference().in(Feet) * ElevatorConstants.kDifSpeedMultiplier;
        rightMotorSpeed = Math.copySign(Math.min(Math.abs(rightMotorSpeed), 1.0), rightMotorSpeed);

        // System.out.println("leftMotorSpeed: " + totalSpeed + ", rightMotorSpeed: " + rightMotorSpeed + ", height: " + getAverageHeight());

        m_leftMotor.set(totalSpeed);
        m_rightMotor.set(rightMotorSpeed);
    }

    
}
