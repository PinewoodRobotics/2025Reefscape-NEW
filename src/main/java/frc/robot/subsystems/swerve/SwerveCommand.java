package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.CustomMath;
import frc.robot.util.controller.FlightStick;
import org.pwrup.util.Vec2;

public class SwerveCommand {

  private static SparkMax driveMotor;
  private static SparkMax turnMotor;

  public static void configureMotors() {
    driveMotor =
      new SparkMax(
        SwerveConstants.kFrontLeftDriveMotorPort,
        MotorType.kBrushless
      );

    turnMotor =
      new SparkMax(
        SwerveConstants.kFrontLeftTurningMotorPort,
        MotorType.kBrushless
      );

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
      .inverted(false)
      .smartCurrentLimit(SwerveConstants.kDriveCurrentLimit);
    driveConfig.encoder.velocityConversionFactor(
      (Math.PI * SwerveConstants.kWheelDiameterMeters) /
      (60 * SwerveConstants.kDriveGearRatio)
    );
    driveMotor.configure(
      driveConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
      .inverted(false)
      .smartCurrentLimit(SwerveConstants.kTurnCurrentLimit);
    turnConfig.closedLoop
      .pid(
        SwerveConstants.kTurnP,
        SwerveConstants.kTurnI,
        SwerveConstants.kTurnD
      )
      .iZone(SwerveConstants.kTurnIZ)
      .positionWrappingEnabled(true)
      .positionWrappingMinInput(-0.5)
      .positionWrappingMaxInput(0.5);
    turnConfig.encoder.positionConversionFactor(
      SwerveConstants.kTurnConversionFactor
    );
    turnMotor.configure(
      turnConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public static RunCommand getDebugMotorRunCommand(
    FlightStick flightStick_left,
    FlightStick flightStick_right
  ) {
    return new RunCommand(() -> {
      driveMotor.set(
        flightStick_left.getRawAxis(FlightStick.AxisEnum.JOYSTICKY.value)
      );

      turnMotor.set(
        flightStick_right.getRawAxis(FlightStick.AxisEnum.JOYSTICKX.value)
      );
    });
  }

  public static RunCommand getManualRunCommand(
    Swerve swerve,
    FlightStick flightStick_left,
    FlightStick flightStick_right
  ) {
    return new RunCommand(
      () -> {
        swerve.drive(
          new Vec2(
            CustomMath.deadband(
              flightStick_right.getRawAxis(
                FlightStick.AxisEnum.JOYSTICKY.value
              ) *
              -1,
              SwerveConstants.kXSpeedDeadband,
              SwerveConstants.kXSpeedMinValue
            ),
            CustomMath.deadband(
              flightStick_right.getRawAxis(
                FlightStick.AxisEnum.JOYSTICKX.value
              ),
              SwerveConstants.kYSpeedDeadband,
              SwerveConstants.kYSpeedMinValue
            )
          ),
          CustomMath.deadband(
            flightStick_right.getRawAxis(
              FlightStick.AxisEnum.JOYSTICKROTATION.value
            ),
            SwerveConstants.kRotDeadband,
            SwerveConstants.kRotMinValue
          ),
          0.2
        );

        swerve.odometryTick();
      },
      swerve
    );
  }
}
