// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.CustomMath;
import frc.robot.util.online.Address;
import frc.robot.util.online.RaspberryPi;
import proto.WatchDogMessage.ProcessType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CameraConstants {

    public static final HashMap<String, SimpleMatrix> cameras = new HashMap<>();

    static {
      cameras.put(
          "front_right",
          CustomMath.fromPose2dToMatrix(
              new Pose2d(0.33, -0.33, new Rotation2d(Math.sqrt(2) / 2, Math.sqrt(2) / 2))));
    }
  }

  public static final class GeneralDebugConstants {

    // Will NOT try to connect to autobahn if true
    public static final boolean kEnableOffline = false;
  }

  public static class PathfindingConstants {

    public static final String mapFilePath = Filesystem.getDeployDirectory().getAbsolutePath() + "/walls.json";
    public static final double rerunDistanceThreshhold = 0.2;
    public static final int maxNodesInRange = 3;
    public static final long maxTimeBetweenPathdfinds = 200;

    // Note that the x should be negative because it is in visible side of the april tag. 
    public static final Pose2d point1 = new Pose2d(
        -1.0,
        0.0,
        new Rotation2d(0));
  }

  public static class AutobahnConstants {

    public static final RaspberryPi tripoli = new RaspberryPi(
        new Address("10.47.65.7", 8080),
        new ProcessType[] {},
        "tripoli");

    public static final RaspberryPi agatha_king = new RaspberryPi(
        new Address("10.47.65.13", 8080),
        new ProcessType[] { ProcessType.CAMERA_PROCESSING },
        "agatha_king");

    public static final RaspberryPi donnager = new RaspberryPi(
        new Address("10.47.65.12", 8080),
        new ProcessType[] { ProcessType.CAMERA_PROCESSING },
        "donnager");

    public static final RaspberryPi[] all = new RaspberryPi[] {
        tripoli,
        agatha_king,
        donnager,
    };
  }

  public static class LocalizationConstants {

    public static final double kMaxDistanceDiffBeforeReset = 0.2;
    public static final double kMaxDegDiffBeforeReset = 30;
    public static final double kMaxTimeMs = 1000;
    public static final String kPosPublicationTopic = "robot/odometry";
  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorPanelPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kFlightPortLeft = 2;
    public static final int kFlightPortRight = 3;
  }

  public static class RobotContainerConstants {

    public static final boolean kSwerveEnabled = true;
    public static final boolean kArmEnabled = true;
  }

  public static class ArmConstants {

    public static final int kArmMotorPort = 32;
    public static final boolean kArmMotorReversed = false;
    public static final int kArmCurrentLimit = 16;

    // Arm PID constants
    public static final double kP = 3; // 2
    public static final double kI = 0.002; // 0.002
    public static final double kD = 4; // 4
    public static final double kIZ = 0;
    public static final double kIZone = 0;
    public static final double kFFCoefficient = 2.4; // 2.4
    public static final double kIMaxAccum = 0.04;

    public static final boolean kEncoderReversed = false;
    public static final double kEncoderOffset = 0.082;
    public static final double kEncoderConversionFactor = 1.0;

    public static final double kArmMinPosition = 0;
    public static final double kArmMaxPosition = 0.3;

    public static final double kArmFlatPosition = 0;
    public static final double kArmDrivingPosition = 0.11; // 0.12
    public static final double kArmScoringPosition = 0.29;
  }

  public static class ClimbArmConstants {

    public static final int kClimbArmMotorPort = 15;
    public static final boolean kClimbArmMotorIsBrushless = false;
    public static final double kClimberArmMotorSpeed = 0.75;
  }

  public static class IntakeConstants {

    public static final String kCanbusAddress = ""; // TEMP
    public static final int kCurrentLimit = 40;

    // Power Distribution Panel Constants
    public static final int kIntakePDPChannel = 4; // TEMP
    public static final double kIntakeCurrentThresholdAmps = 7; // TEMP: whether the intake is intaking a note

    // Motor Speeds
    public static final double kIntakeSpeed = 0.5; // TEMP
    public static final double kOutputSpeed = 0.7; // TEMP

    // Intake Paramters
    public static final int kIntakePort = 30; // TEMP
    public static final int kShooterPort1 = 1; // TEMP
    public static final int kShooterPort2 = 2; // TEMP
    public static final boolean kIsBrushless = false; // TEMP

    public static final boolean kIsIntakeReversed = false; // TEMP
    public static final boolean kIsShooter1Reversed = false; // TEMP
    public static final boolean kIsShooter2Reversed = true; // TEMP
  }

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
    public static final int kFrontLeftDriveMotorPort = 25;
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kRearLeftDriveMotorPort = 12;
    public static final int kRearRightDriveMotorPort = 21;

    // whether the driving encoders are flipped
    public static final InvertedValue kFrontLeftDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearLeftDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kFrontRightDriveMotorReversed = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRearRightDriveMotorReversed = InvertedValue.Clockwise_Positive;

    // the turning motor ports
    public static final int kFrontLeftTurningMotorPort = 20;
    public static final int kFrontRightTurningMotorPort = 23;
    public static final int kRearLeftTurningMotorPort = 13;
    public static final int kRearRightTurningMotorPort = 22;

    // whether the turning enoders are flipped
    public static final InvertedValue kFrontLeftTurningMotorReversed = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kFrontRightTurningMotorReversed = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kRearLeftTurningMotorReversed = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kRearRightTurningMotorReversed = InvertedValue.CounterClockwise_Positive;

    // the CANCoder turning encoder ports - updated 2/12/24
    public static final int kFrontLeftCANcoderPort = 2;
    public static final int kFrontRightCANcoderPort = 3;
    public static final int kRearLeftCANcoderPort = 1;
    public static final int kRearRightCANcoderPort = 4;

    // whether the turning CANCoders are flipped

    public static final SensorDirectionValue kFrontLeftCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
    public static final SensorDirectionValue kFrontRightCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
    public static final SensorDirectionValue kRearLeftCANcoderDirection = SensorDirectionValue.Clockwise_Positive;
    public static final SensorDirectionValue kRearRightCANcoderDirection = SensorDirectionValue.Clockwise_Positive;

    // magnetic offset for the CANCoders
    // you can find these by connecting to the RoboRIO by USB on the drive station,
    // opening the Phoenix Tuner app, and taking snapshots of
    // the rotational values of the CANCoders while in they are in the forward state
    public static final double kFrontLeftCANcoderMagnetOffset = 0.328;
    public static final double kFrontRightCANcoderMagnetOffset = 0.347;
    public static final double kRearLeftCANcoderMagnetOffset = -0.192;
    public static final double kRearRightCANcoderMagnetOffset = 0.499;

    // stats used by SwerveSubsystem for math
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveBaseWidth = 0.76;
    public static final double kDriveBaseLength = 0.76;

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
    public static final double kTurnP = 1.5;
    public static final double kTurnI = 0.0015;
    public static final double kTurnD = 0.12;
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

  public static class VisionConstants {

    // All units are in meters

    // Constants such as camera and target height stored. Change per robot and goal!
    public static final double kCAMERA_HEIGHT_METERS = Units.inchesToMeters(24);

    public static final double kTARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    public static final double kCAMERA_PITCH_RADIANS = Units.degreesToRadians(
        0);
    public static final double kCAMERA_PITCH = Units.degreesToRadians(35);

    // How far from the target we want to be

    public static final double kAmpXGoal = 0;
    public static final double kAmpYGoal = 0.8;
    public static final double kAmpRotGoal = 0;

    public static final double kXP = 0.8; // 0.8
    public static final double kXI = 0.001;
    public static final double kXD = 0.0;

    public static final double kYP = 0.8; // 0.8
    public static final double kYI = 0.01;
    public static final double kYD = 0.0;

    public static final double kRP = 1.2; // 1.2
    public static final double kRI = 0.01;
    public static final double kRD = 0.0;
  }

  public static class FieldConstants {

    public static final double kRedAmpXPosition = 0; // TEMP WE NEED TO FIND THIS IN METERS
    public static final double kRedAmpYPosition = 0;

    public static final double kBlueAmpXPosition = 0; // TEMP WE NEED TO FIND THIS IN METERS
    public static final double kBlueAmpYPosition = 0;
  }

  public static class LEDConstants {

    public static final int canifierPort = 5;
  }
}
