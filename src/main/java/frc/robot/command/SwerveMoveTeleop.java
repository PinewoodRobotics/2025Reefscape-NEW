package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import frc.robot.util.controller.FlightStick;
import org.pwrup.util.Vec2;

public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightStick m_flightStick_right;

  public SwerveMoveTeleop(
    SwerveSubsystem swerveSubsystem,
    FlightStick flightStick_right
  ) {
    m_swerveSubsystem = swerveSubsystem;
    m_flightStick_right = flightStick_right;
    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void execute() {
    m_swerveSubsystem.drive(
      new Vec2(
        CustomMath.deadband(
          m_flightStick_right.getRawAxis(FlightStick.AxisEnum.JOYSTICKY.value),
          SwerveConstants.kXSpeedDeadband,
          SwerveConstants.kXSpeedMinValue
        ),
        CustomMath.deadband(
          m_flightStick_right.getRawAxis(FlightStick.AxisEnum.JOYSTICKX.value) *
          -1,
          SwerveConstants.kYSpeedDeadband,
          SwerveConstants.kYSpeedMinValue
        )
      ),
      CustomMath.deadband(
        m_flightStick_right.getRawAxis(
          FlightStick.AxisEnum.JOYSTICKROTATION.value
        ) *
        -1,
        SwerveConstants.kRotDeadband,
        SwerveConstants.kRotMinValue
      ),
      0.2
    );
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }
}
