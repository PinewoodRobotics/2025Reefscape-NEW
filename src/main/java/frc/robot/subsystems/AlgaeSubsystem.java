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

    
    public AlgaeSubsystem() {
        configureMotors();

       
        
    }

    private void configureMotors() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(AlgaeConstants.kLeftMotorInverted);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        m_leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(AlgaeConstants.kRightMotorInverted);
        rightMotorConfig.idleMode(IdleMode.kBrake);
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

}
