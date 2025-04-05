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

  public final static Distance kStartingHeight = Distance.ofRelativeUnits(0.9, Feet); //0.9
  public final static Distance kMinHeight = kStartingHeight;
  public final static Distance kMaxHeight = Distance.ofRelativeUnits(4.3, Feet).plus(kStartingHeight); //5.2
  public final static Distance kRestingHeight = Distance.ofRelativeUnits(0.3, Feet).plus(kStartingHeight); //1.2
  public final static Distance kIntakeHeight = kMinHeight;
  public final static Distance kL2Height = Distance.ofRelativeUnits(0.9, Feet).plus(kStartingHeight); //1.9
  public final static Distance kL3Height = Distance.ofRelativeUnits(1.95, Feet).plus(kStartingHeight); //2.85
  public final static Distance kL4Height = kMaxHeight;
  public final static Distance kMidAlgaeHeight = Distance.ofRelativeUnits(2.65, Feet).plus(kStartingHeight); //3.55
  public final static Distance kHighAlgaeHeight = Distance.ofRelativeUnits(3.85, Feet).plus(kStartingHeight); //4.75
  public final static Distance kProcessorHeight = Distance.ofRelativeUnits(1.1, Feet).plus(kStartingHeight); //2
  public final static Distance kDefaultHeight = kMinHeight;
  public final static Distance kReefClearanceHeight = Distance.ofRelativeUnits(2, Feet).plus(kStartingHeight);
}
