package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeConstants {

  public static final int leftMotorID = 28;
  public static final int rightMotorID = 26;

  public static final int kCurrentLimit = 20;

  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = !kLeftMotorInverted;

  public static final double kIntakeSpeed = 0.5;
  public static final double kEjectSpeed = -0.2;
  public static final double kHoldSpeed = 0.05;

  public static final int wristMotorID = 15;
  public static final int kWristCurrentLimit = 30;
  public static final boolean kMotorInverted = false;
  public static final boolean kAbsoluteEncoderInverted = true;

  public static final double kTolerance = 0.014;

  public static final double kGearingRatio = 1.0 / 100;

  public static final double kP = 2.5;
  public static final double kI = 0.002;
  public static final double kD = 0.5;
  public static final double kFF = 0.27;
  public static final double kIZone = 0;

  public static final Rotation2d kWristOffset = Rotation2d.fromRotations(0.866);

    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d kIntakeAngle = kMinAngle;
    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(0.225);
    public static final Rotation2d kWristDefaultAngle = Rotation2d.fromRotations(0.215);
}
