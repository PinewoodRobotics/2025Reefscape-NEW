package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralConstants {
    
    public final static int wristMotorID = 16;
    public final static int intakeMotorID = 36;

    public final static double kIntakeSpeed = 0.2;
    public final static boolean kIntakeInverted = true;

    public final static int kAmpLimit = 5;
    public final static Rotation2d kWristOffset = Rotation2d.fromRotations(0.36); //offset to when the flat part of the arm is parallel to the ground
    public final static Rotation2d kWristMinPosition = Rotation2d.fromDegrees(90);
    public final static Rotation2d kWristMaxPosition = Rotation2d.fromDegrees(270);
    public final static int kGearingRatio = 16;

    public final static boolean kInverted = false;
    public final static double kP = 0.8;
    public final static double kI = 0.002;
    public final static double kIZone = 0.03;
    public final static double kD = 1;
    public final static double kFF = 0;
}
