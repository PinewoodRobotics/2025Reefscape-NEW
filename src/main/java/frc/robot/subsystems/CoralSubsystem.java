package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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

public class CoralSubsystem extends SubsystemBase {
    private SparkMax wrist = new SparkMax(CoralConstants.wristMotorID, MotorType.kBrushless);
    
    private SparkFlex intake = new SparkFlex(CoralConstants.intakeMotorID, MotorType.kBrushless);

    public CoralSubsystem() {
        configureWrist();
        configureIntake();
    }

    private void configureIntake() {
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig.inverted(CoralConstants.kIntakeInverted);
        intakeConfig.idleMode(IdleMode.kBrake);

        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureWrist() {
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.smartCurrentLimit(CoralConstants.kAmpLimit);
        wristConfig.inverted(CoralConstants.kInverted);
        wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        wristConfig.closedLoop.pid(
            CoralConstants.kP,
            CoralConstants.kI,
            CoralConstants.kD
        )
        .iZone(CoralConstants.kIZone);
        
        wristConfig.absoluteEncoder.zeroOffset(CoralConstants.kWristOffset);
        


        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake() {
        runIntake(CoralConstants.kIntakeSpeed);
    }

    public void runIntake(double speed) {
        intake.set(speed);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void setWristPosition(Rotation2d position) {
        wrist.getClosedLoopController().setReference(position.getRotations(), ControlType.kPosition);
        System.out.println("moving the wrist!");
    }

    public Rotation2d getWristPosition() {
        return Rotation2d.fromRotations(wrist.getAbsoluteEncoder().getPosition());
    }

    
}
