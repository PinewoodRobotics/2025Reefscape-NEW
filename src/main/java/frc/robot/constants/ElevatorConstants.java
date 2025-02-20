package frc.robot.constants;

public class ElevatorConstants {
    
    public final static int leftMotorID = 27;
    public final static int rightMotorID = 14;

    public final static boolean kLeftMotorInverted = false; //"left" as seen from the back of the robot
    public final static boolean kRightMotorInverted = !kLeftMotorInverted;

    public final static double kGearRatio = 1.0 / 5;
    public final static double kAxleToHeightRatio = 0.91613; //found experimentally
    public final static double kGearHeightRatio = kGearRatio * kAxleToHeightRatio;
    
    
    public final static double kStartingHeight = 0.90625;
    public final static double kMaxHeight = 0;

    public final static boolean kSetpointRamping = false;
    public final static double kMaxSetpointRamp = 0.2;

    public final static double kP = 0.1;
    public final static double kI = 0.003;
    public final static double kD = 0;
    public final static double kFF = 0;
    public final static double kIZone = Double.POSITIVE_INFINITY;
    public final static double kDifSpeedMultiplier = 0;

}
