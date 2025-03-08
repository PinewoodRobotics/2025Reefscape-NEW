package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.MathFunc;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class ElevatorSubsystem extends SubsystemBase {
    
    //LEFT MOTOR IS THE LEADER
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;
    private PIDController m_pid;
    private ElevatorFeedforward m_feedforward;

    private double m_currentSetpoint = ElevatorConstants.kStartingHeight;
    private boolean atTarget = false;

    public ElevatorSubsystem() {
        m_leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
        m_rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

        m_pid = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
        );

        m_pid.setTolerance(0.5);

        configureMotors();

        m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
    }

    private void configureMotors() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(ElevatorConstants.kLeftMotorInverted) 
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);
            
        m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight);
        

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(ElevatorConstants.kRightMotorInverted)
            .follow(m_leftMotor, true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);

        m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight);

        m_pid.setIZone(ElevatorConstants.kIZone);
        
    }

    public void setHeight(Distance height) {
         m_currentSetpoint = height.in(Feet);
    }

    public Distance getAverageHeight() {
        double height = (m_leftMotor.getEncoder().getPosition() + m_rightMotor.getEncoder().getPosition()) / 2.0;
        return Distance.ofRelativeUnits(height, Feet);
    }

    public double calculateSpeed(double setpoint) {
        double motorPowerPid = m_pid.calculate(getAverageHeight().in(Feet), setpoint);
        double ff = calculateFeedForwardValue(m_feedforward);
        return MathUtil.clamp(motorPowerPid + ff, -1, 1);
    }

    public boolean atTarget() {
        double tolerance = 0.3;
        double currentPosition = getAverageHeight().in(Feet);
        if(Math.abs(currentPosition - m_currentSetpoint) <= tolerance) {
            return !atTarget;
        }
        return atTarget;
    }
    
    public void atLimit() {
        double currentHeight = getAverageHeight().in(Feet);
        if (currentHeight >= ElevatorConstants.kMaxHeight) {
            stopMotors();
        }

        else if (currentHeight < ElevatorConstants.kStartingHeight) {
            stopMotors();
            
        }
    }

    public void stopMotors(){
        m_leftMotor.set(0); 
    }

    
    public double calculateFeedForwardValue(ElevatorFeedforward feedforward){
        double currentVelocity = m_leftMotor.getEncoder().getVelocity();
        return feedforward.calculate(currentVelocity);
        


    }
    


    /**
     * 
     * @return The distance between the two motors (left - right)
     */
    /*public Distance getHeightDifference() {
        return Distance.ofRelativeUnits(m_leftMotor.getEncoder().getPosition() - m_rightMotor.getEncoder().getPosition(), Feet);
    }
    */
    
    /**
     *  Calculates speeds using the pid controller. Leaves the left motor speed alone, and then
     * adjusts the right motors speed based on how far apart the motors are
     * 
     */    
    @Override
    public void periodic() {
        // m_currentSetpoint = MathFunc.rampSetpoint(m_setpoint, m_currentSetpoint, ElevatorConstants.kMaxSetpointRamp);
        System.out.println("setpoint: " + m_currentSetpoint);
        //double totalSpeed = m_pid.calculate(getAverageHeight().in(Feet), m_currentSetpoint);
        

        //double rightMotorSpeed = totalSpeed + getHeightDifference().in(Feet) * ElevatorConstants.kDifSpeedMultiplier;
        //double rightMotorSpeed = MathUtil.clamp(totalSpeed, -1, 1);

        // System.out.println("leftMotorSpeed: " + totalSpeed + ", rightMotorSpeed: " + rightMotorSpeed + ", height: " + getAverageHeight());

        m_leftMotor.set(calculateSpeed(m_currentSetpoint));
        atLimit();
        //m_rightMotor.set(calculateSpeed(m_currentSetpoint));
    }   
}

