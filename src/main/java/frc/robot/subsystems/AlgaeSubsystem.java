package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    
    private SparkMax m_leftMotor = new SparkMax(AlgaeConstants.leftMotorID, MotorType.kBrushless);
    private SparkMax m_rightMotor = new SparkMax(AlgaeConstants.rightMotorID, MotorType.kBrushless);

    public AlgaeSubsystem() {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(AlgaeConstants.kLeftMotorInverted);

        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(AlgaeConstants.kRightMotorInverted);

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
