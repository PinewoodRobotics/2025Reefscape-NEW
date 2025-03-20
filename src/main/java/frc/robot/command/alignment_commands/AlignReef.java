package frc.robot.command.alignment_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignReef extends Command {
  
  private SwerveSubsystem m_swerveSubsystem;
  private SwerveMoveTeleop m_moveCommand;

  public AlignReef(
    SwerveSubsystem swerveSubsystem,
    SwerveMoveTeleop moveCommand
  ) {
    m_moveCommand = moveCommand;
    m_swerveSubsystem = swerveSubsystem;
  }

  @Override
  public void initialize() {
    pickSide();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  private void pickSide() {
    double heading = m_swerveSubsystem.getGlobalGyroAngle();
    System.out.println("Gyro angle: " + m_swerveSubsystem.getGlobalGyroAngle());

    if (heading < -150 || heading > 150) {
      m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(180));
    } else if (heading > 90 && heading < 150) {
      m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(120));
    } else if (heading > 30 && heading < 90) {
      m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(60));
    } else if (heading > -30 && heading < 30) {
      m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(0));
    } else if (heading > -90 && heading < -30) {
      m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(-60));
    } else if (heading > -150 && heading < -90) {
      m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(-120));
    }
  }

}
