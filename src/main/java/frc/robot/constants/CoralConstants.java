package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralConstants {

  public final static int wristMotorID = 19;
  public final static int intakeMotorID = 26;

  public final static double kIntakeSpeed = 0.2;
  public final static boolean kIntakeInverted = true;

  public final static int kAmpLimit = 5;
  public final static Rotation2d kWristOffset = Rotation2d.fromRotations(0.77); //offset to when the flat part of the arm is parallel to the ground
  public final static Rotation2d kWristMinPosition = Rotation2d.fromDegrees(90);
  public final static Rotation2d kWristMaxPosition = Rotation2d.fromDegrees(270);
  public final static double kGearingRatio = 1 / 9.0 * 16 / 40;

  public final static boolean kInverted = false;
  public final static double kP = 1; //1
  public final static double kI = 0;
  public final static double kIZone = 0.03; //0.03
  public final static double kD = 0.3; //0.3

  public final static double kFF = 0.2;
  public final static Rotation2d kFFOffset = Rotation2d.fromRotations(-0.358);
}
