package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    
    public final static int leftMotorID = 14;
    public final static int rightMotorID = 27;

    public final static boolean kLeftMotorInverted = false; //"left" as seen from the back of the robot
    public final static boolean kRightMotorInverted = !kLeftMotorInverted;

    public final static double kGearRatio = 1.0 / 15;
    public final static double kAxleToHeightRatio = 0.91613; //found experimentally
    public final static double kGearHeightRatio = kGearRatio * kAxleToHeightRatio;
    
    public final static boolean kSetpointRamping = true;
    public final static double kMaxSetpointRamp = 0.15;

    public final static double kP = 0.6;
    public final static double kI = 0.08;
    public final static double kD = 0;
    public final static double kIZone = Double.POSITIVE_INFINITY;
    public final static double kDifSpeedMultiplier = 0;
    public final static double kS = 0;
    public final static double kV = 0;
    public final static double kG = 0.02;
    public final static double kA = 0;
    public final static double kTolerance = 0.1;
    
    public final static Distance kStartingHeight = Distance.ofRelativeUnits(0.9, Feet);
    public final static Distance kMinHeight = kStartingHeight;
    public final static Distance kDefaultHeight = Distance.ofRelativeUnits(1.2, Feet);
    public final static Distance kL2Height = Distance.ofRelativeUnits(1.9, Feet);
    public final static Distance kL3Height = Distance.ofRelativeUnits(2.85, Feet);
    public final static Distance kL4Height = Distance.ofRelativeUnits(5.2, Feet);
    public final static Distance kMidAlgaeHeight = Distance.ofRelativeUnits(3.55, Feet);
    public final static Distance kHighAlgaeHeight = Distance.ofRelativeUnits(4.75, Feet);
    public final static Distance kProcessorHeight = Distance.ofRelativeUnits(2, Feet);
    public final static Distance kMaxHeight = Distance.ofRelativeUnits(5.2, Feet);

}
