package frc.robot.command;

import java.util.function.Supplier;

import org.pwrup.util.Vec2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import pwrup.frc.core.controller.FlightModule;
import pwrup.frc.core.controller.FlightStick;

public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightModule controller;

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;

    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void execute() {
    double r = CustomMath.deadband(
        controller.leftFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKROTATION.value) * -1,
        SwerveConstants.kRotDeadband,
        SwerveConstants.kRotMinValue);

    double x = CustomMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKY.value) * -1,
        SwerveConstants.kXSpeedDeadband,
        SwerveConstants.kXSpeedMinValue);

    double y = CustomMath.deadband(
        controller.rightFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKX.value) *
            -1,
        SwerveConstants.kYSpeedDeadband,
        SwerveConstants.kYSpeedMinValue);

    var velocity = SwerveSubsystem.fromPercentToVelocity(new Vec2(x, y), r);
    m_swerveSubsystem.drive(velocity, SwerveSubsystem.DriveType.GYRO_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stop();
  }
}
