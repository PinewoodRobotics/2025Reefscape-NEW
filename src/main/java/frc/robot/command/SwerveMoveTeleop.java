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
  private boolean m_headingControl = false;
  private Rotation2d m_headingSetpoint = Rotation2d.fromDegrees(0);
  private PIDController m_headingPID = new PIDController(
      SwerveConstants.kHeadingP,
      SwerveConstants.kHeadingI,
      SwerveConstants.kHeadingD);
  private Supplier<Boolean> isNonRelative;

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller, Supplier<Boolean> isNonRelative) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    m_headingPID.enableContinuousInput(-0.5, 0.5);
    this.isNonRelative = isNonRelative;

    addRequirements(m_swerveSubsystem);
  }

  @Override
  public void execute() {
    double joystickRotation = CustomMath.deadband(
        controller.leftFlightStick.getRawAxis(
            FlightStick.AxisEnum.JOYSTICKROTATION.value) *
            -1,
        SwerveConstants.kRotDeadband,
        SwerveConstants.kRotMinValue);

    if (Math.abs(joystickRotation) > 0) {
      m_headingControl = false;
    }

    double r;
    if (m_headingControl) {
      r = -m_headingPID.calculate(m_swerveSubsystem.getGlobalGyroAngle() / 360);
    } else {
      r = joystickRotation;
    }

    if (!isNonRelative.get()) {
      m_swerveSubsystem.drive(
          new Vec2(
              CustomMath.deadband(
                  controller.rightFlightStick.getRawAxis(
                      FlightStick.AxisEnum.JOYSTICKY.value),
                  SwerveConstants.kXSpeedDeadband,
                  SwerveConstants.kXSpeedMinValue),
              CustomMath.deadband(
                  controller.rightFlightStick.getRawAxis(
                      FlightStick.AxisEnum.JOYSTICKX.value) *
                      -1,
                  SwerveConstants.kYSpeedDeadband,
                  SwerveConstants.kYSpeedMinValue)),
          r,
          SwerveConstants.kDefaultSpeedMultiplier);
    } else {
      m_swerveSubsystem.driveRaw(
          new Vec2(
              CustomMath.deadband(
                  controller.rightFlightStick.getRawAxis(
                      FlightStick.AxisEnum.JOYSTICKY.value),
                  SwerveConstants.kXSpeedDeadband,
                  SwerveConstants.kXSpeedMinValue),
              CustomMath.deadband(
                  controller.rightFlightStick.getRawAxis(
                      FlightStick.AxisEnum.JOYSTICKX.value) *
                      -1,
                  SwerveConstants.kYSpeedDeadband,
                  SwerveConstants.kYSpeedMinValue)),
          r,
          SwerveConstants.kDefaultSpeedMultiplier);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }

  public void setHeadingControl(Rotation2d heading) {
    m_headingControl = true;
    m_headingPID.setSetpoint(heading.getRotations());
    m_headingSetpoint = heading;
  }

  public Command setHeadingControlCommand(Rotation2d heading) {
    return new InstantCommand(() -> setHeadingControl(heading));
  }

  public Rotation2d getHeadingControl() {
    return m_headingSetpoint;
  }
}
