package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.FlightStick;

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

  /*
  @Override
  public void execute() {
    m_swerveSubsystem.drive(
        new Vec2(
            controller.getLeftX(),
            controller.getLeftY()),
        controller.getRightX(),
        0.2);
  }
  */

  @Override
  public void execute() {
    m_swerveSubsystem.drive(
        new Vec2(
            CustomMath.deadband(
                controller.rightFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKY.value),
                SwerveConstants.kXSpeedDeadband,
                SwerveConstants.kXSpeedMinValue),
            CustomMath.deadband(
                controller.rightFlightStick.getRawAxis(FlightStick.AxisEnum.JOYSTICKX.value) *
                    -1,
                SwerveConstants.kYSpeedDeadband,
                SwerveConstants.kYSpeedMinValue)),
        CustomMath.deadband(
            controller.leftFlightStick.getRawAxis(
                FlightStick.AxisEnum.JOYSTICKROTATION.value) *
                -1,
            SwerveConstants.kRotDeadband,
            SwerveConstants.kRotMinValue),
        SwerveConstants.kDefaultSpeedMultiplier);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }
}
