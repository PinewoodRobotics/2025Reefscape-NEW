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


public class ElevatorSubsystem extends SubsystemBase {
    
    //LEFT MOTOR IS THE LEADER
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;
    private PIDController m_pid;
    private ElevatorFeedforward m_feedforward;

    private Distance m_setpoint = ElevatorConstants.kStartingHeight;
    private Distance m_currentSetpoint = ElevatorConstants.kStartingHeight;

    public ElevatorSubsystem() {
        m_leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
        m_rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

        m_pid = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
        );
        m_pid.setTolerance(ElevatorConstants.kTolerance);
        m_feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
    

        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(ElevatorConstants.kLeftMotorInverted) 
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);
            
        m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight.in(Feet));


        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(ElevatorConstants.kRightMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .encoder.positionConversionFactor(ElevatorConstants.kGearHeightRatio);

        m_rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.getEncoder().setPosition(ElevatorConstants.kStartingHeight.in(Feet));

        m_pid.setIZone(ElevatorConstants.kIZone);
    }

    public void setHeight(Distance height) {
        m_setpoint = height;
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
        return m_pid.atSetpoint();
    }

    public void stopMotors(){
        m_leftMotor.stopMotor(); 
        m_rightMotor.stopMotor();
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
    
    private int m_count = 0;
    /**
     *  Calculates speeds using the pid controller. Leaves the left motor speed alone, and then
     * adjusts the right motors speed based on how far apart the motors are
     * 
     */    
    @Override
    public void periodic() {
        if (ElevatorConstants.kSetpointRamping) {
            m_currentSetpoint = Distance.ofRelativeUnits(MathFunc.rampSetpoint(m_setpoint.in(Feet), m_currentSetpoint.in(Feet), ElevatorConstants.kMaxSetpointRamp), Feet);
        } else {
            m_currentSetpoint = m_setpoint;
        }
        
        double speed = calculateSpeed(m_currentSetpoint.in(Feet));
        
        m_leftMotor.setVoltage(speed * 12);
        m_rightMotor.setVoltage(speed * 12);
        

        m_count++;
        if (m_count == 3) {
            // System.out.println("LeftCurrent: " + m_leftMotor.getOutputCurrent() + ", RightCurrent: " + m_rightMotor.getOutputCurrent());
            System.out.print("Position: " + getAverageHeight());
            System.out.println(", Leftpower:" + m_leftMotor.getAppliedOutput() + ", Rightpower: " + m_rightMotor.getAppliedOutput());
            
            m_count = 0;
        }
    }


    
}

