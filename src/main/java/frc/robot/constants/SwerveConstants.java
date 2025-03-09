package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {
  public static final Translation2d rearLeftTranslation = new Translation2d(
        0.38,
        0.38);

    public static final Translation2d rearRightTranslation = new Translation2d(
        0.38,
        -0.38);

    public static final Translation2d frontRightTranslation = new Translation2d(
        -0.38,
        -0.38);

    public static final Translation2d frontLeftTranslation = new Translation2d(
        -0.38,
        0.38);

    // the driving motor ports
    public static final int kFrontLeftDriveMotorPort = 7;
    public static final int kFrontRightDriveMotorPort = 9;
    public static final int kRearLeftDriveMotorPort = 11;
    public static final int kRearRightDriveMotorPort = 13;

    // whether the driving encoders are flipped
    public static final InvertedValue kFrontLeftDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearLeftDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kFrontRightDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearRightDriveMotorReversed = InvertedValue.Clockwise_Positive   ;

    // the turning motor ports
    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kFrontRightTurningMotorPort = 8;
    public static final int kRearLeftTurningMotorPort = 10;
    public static final int kRearRightTurningMotorPort = 12;

    // whether the turning enoders are flipped
    public static final InvertedValue kFrontLeftTurningMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kFrontRightTurningMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearLeftTurningMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearRightTurningMotorReversed = InvertedValue.Clockwise_Positive;

    // the CANCoder turning encoder ports - updated 2/12/24
    public static final int kFrontLeftCANcoderPort = 2;
    public static final int kFrontRightCANcoderPort = 3;
    public static final int kRearLeftCANcoderPort = 4;
    public static final int kRearRightCANcoderPort = 5;

    // whether the turning CANCoders are flipped

    public static final SensorDirectionValue kFrontLeftCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
    public static final SensorDirectionValue kFrontRightCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
    public static final SensorDirectionValue kRearLeftCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
    public static final SensorDirectionValue kRearRightCANcoderDirection = SensorDirectionValue.Clockwise_Positive;

    // magnetic offset for the CANCoders
    // you can find these by connecting to the RoboRIO by USB on the drive station,
    // opening the Phoenix Tuner app, and taking snapshots of
    // the rotational values of the CANCoders while in they are in the forward state
    // units: rotations
    public static final double kFrontLeftCANcoderMagnetOffset = 0.056885;
    public static final double kFrontRightCANcoderMagnetOffset = -0.064;
    public static final double kRearLeftCANcoderMagnetOffset = 0.253;
    public static final double kRearRightCANcoderMagnetOffset = 0.111;

    // stats used by SwerveSubsystem for math
    public static final double kWheelDiameterMeters = 0.15; //TEMP
    public static final double kDriveBaseWidth = 0.66;
    public static final double kDriveBaseLength = 0.66;

    // stats used by SwerveSubsystem for deadbanding
    public static final double kXSpeedDeadband = 0.05;
    public static final double kXSpeedMinValue = 0;
    public static final double kYSpeedDeadband = 0.05;
    public static final double kYSpeedMinValue = 0;
    public static final double kRotDeadband = 0.05;
    public static final double kRotMinValue = 0;

    public static final boolean kFieldRelative = true;
    public static final boolean kOptimizeAngles = true;
    public static final boolean kPIDDirection = true;
    public static final double kDirectionP = 2;
    public static final double kDirectionI = 0.004;
    public static final double kDirectionD = 0.02;
    public static final double kDirectionMultiplier = 0.01;

    // PID values for the driving
    public static final double kDriveP = 0.01;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveIZ = 0;
    public static final double kDriveFF = 0;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    // multiplies the output speed of all of the drive motors, ALWAYS (0, 1).
    public static final double kDefaultSpeedMultiplier = 1.0;
    public static final double kRotationSpeedMultiplier = 0.5;
    public static final double kIntakeSpeedMultiplier = kDefaultSpeedMultiplier;
    public static final double kAutonSpeedMultiplier = 0.5;

    public static final double kDriveMaxRPM = 5700;
    public static final double kDriveStatorLimit = 30; //TEMP
    public static final double kDriveSupplyLimit = 30; //TEMP

    // PID values for the turning
    public static final double kTurnP = 20;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0;
    public static final double kTurnIZ = 0;
    public static final double kTurnFF = 0;
    public static final double kTurnMinOutput = -1;
    public static final double kTurnMaxOutput = 1;
    public static final int kTurnStatorLimit = 30; //TEMP
    public static final double kTurnSupplyLimit = 30; //TEMP
    // because the turn gearing ratio is not 1:1, we need to spin the motor many
    // times to equal one spin of the module
    // this constant is used for the position conversion factor. (every 150 turns of
    // motors is 7 rotations of the module)
    public static final double kTurnConversionFactor = 25.9;

    // because the drive gearing ratio is not 1:1, we need to spin the motor many
    // times to equal one spin of the module
    public static final double kDriveGearRatio = 4.94;
}
