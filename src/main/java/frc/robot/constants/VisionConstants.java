package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double kCAMERA_HEIGHT_METERS = Units.inchesToMeters(24);

    public static final double kTARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    public static final double kCAMERA_PITCH_RADIANS = Units.degreesToRadians(
      0);
    public static final double kCAMERA_PITCH = Units.degreesToRadians(35);

    // How far from the target we want to be

    public static final double kAmpXGoal = 1;
    public static final double kAmpYGoal = 1;
    public static final double kAmpRotGoal = 0;
    public static final double kXP = 0.2;
    public static final double kXD = 0.0;
    public static final double kYP = 0.2;
    public static final double kYD = 0.0;
    public static final double kRP = 0.2;
    public static final double kRD = 0.0;
    // public static final double kLINEAR_P = 0.2;

    // public static final double kLINEAR_D = 0.0;

    // public static final double kANGULAR_P = 0.2;

    // public static final double kANGULAR_D = 0.0;
}
