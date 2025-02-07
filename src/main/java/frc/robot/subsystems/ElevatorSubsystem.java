package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private SparkMax m_leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
    private SparkMax m_rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

    public ElevatorSubsystem() {
        
    }

    public void configureMotors() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    }

    public void setHeight() {

    }

    public void getHeight() {

    }

    
}
