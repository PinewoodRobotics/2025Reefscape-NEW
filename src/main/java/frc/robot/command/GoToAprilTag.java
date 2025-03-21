package frc.robot.command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.controller.LogitechController;

public class GoToAprilTag extends Command {

  VisionSubsystem m_vision;
  SwerveSubsystem m_swerveSubsystem;

  double m_desiredX, m_desiredY, m_desiredRot;
  Joystick m_controller;

  public GoToAprilTag(
    Joystick controller,
    SwerveSubsystem swerveDrive,
    VisionSubsystem vision,
    double x,
    double y,
    double rot
  ) {
    m_vision = vision;
    m_swerveSubsystem = swerveDrive;
    m_controller = controller;

    m_desiredX = x;
    m_desiredY = y;
    m_desiredRot = rot;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var target = m_vision.findBestTarget();
    if (target != null) {
      double sidewaysSpeedX = m_vision.calculateSidewaysSpeedX(
        target,
        m_desiredX
      );
      double forwardSpeedY = m_vision.calculateForwardSpeedY(
        target,
        m_desiredY
      );
      double rotationSpeed = m_vision.calculateRotationSpeed(
        target,
        m_desiredRot
      );
      System.out.print("sidewaysSpeedX, forwardSpeedY, rotationSpeed: " + sidewaysSpeedX + ", " + forwardSpeedY + ", " + rotationSpeed + "\n");
    //   m_swerveSubsystem.drive(sidewaysSpeedX, forwardSpeedY, rotationSpeed);
    //   m_swerveSubsystem.driveRaw(
    //     new org.pwrup.util.Vec2(sidewaysSpeedX, forwardSpeedY),
    //     rotationSpeed,
    //     0.2
    //   );
    } else {
      double x = m_controller.getX();
      double y = m_controller.getY();
      double z = m_controller.getZ();
      System.out.print("x, y, z: " + x + ", " + y + ", " + z + "\n");
    //   m_swerveSubsystem.joystickDrive(x, -y, z);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
