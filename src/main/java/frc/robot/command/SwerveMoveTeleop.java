package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OldConstants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;

public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final XboxController controller;

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      XboxController controller) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void execute() {
    m_swerveSubsystem.drive(
        new Vec2(
            CustomMath.deadband(-controller.getLeftY(), 0.2,
                SwerveConstants.kXSpeedMinValue),
            CustomMath.deadband(controller.getLeftX(), 0.2,
                SwerveConstants.kXSpeedMinValue)),
        CustomMath.deadband(controller.getRightX(), 0.2, 0),
        0.2);
  }

  /*
  @Override
  public void execute() {
  m_swerveSubsystem.drive(
  new Vec2(
     CustomMath.deadband(
         controller.getRawAxis(FlightStick.AxisEnum.JOYSTICKY.value) * -1,
         SwerveConstants.kXSpeedDeadband,
         SwerveConstants.kXSpeedMinValue),
     CustomMath.deadband(
         controller.getRawAxis(FlightStick.AxisEnum.JOYSTICKX.value),
         SwerveConstants.kYSpeedDeadband,
         SwerveConstants.kYSpeedMinValue)),
  CustomMath.deadband(
     controller.getRawAxis(
         FlightStick.AxisEnum.JOYSTICKROTATION.value),
     SwerveConstants.kRotDeadband,
     SwerveConstants.kRotMinValue),
  0.2);
  } */

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }
}
