package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.hardware.AHRSGyro;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath;
import frc.robot.util.controller.FlightModule;
import frc.robot.util.controller.FlightStick;

public class SwerveMoveTeleop extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final FlightModule controller;
  private boolean m_headingControl = false;
  private Rotation2d m_headingSetpoint = Rotation2d.fromDegrees(0);
  private PIDController m_headingPID = new PIDController(
    SwerveConstants.kHeadingP,
    SwerveConstants.kHeadingI,
    SwerveConstants.kHeadingD
  );

  public SwerveMoveTeleop(
      SwerveSubsystem swerveSubsystem,
      FlightModule controller
  ) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.controller = controller;
    m_headingPID.enableContinuousInput(-0.5, 0.5);

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
    // System.out.println("setpoint: " + m_headingPID.getSetpoint() + " Gyro angle: " + m_swerveSubsystem.getGlobalGyroAngle() / 360 + " angleMath: " + m_headingPID.calculate(m_swerveSubsystem.getGlobalGyroAngle() / 360));

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
        r,
        SwerveConstants.kDefaultSpeedMultiplier);
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
