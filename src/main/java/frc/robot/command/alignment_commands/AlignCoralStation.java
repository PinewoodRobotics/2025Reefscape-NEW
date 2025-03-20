package frc.robot.command.alignment_commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.command.SwerveMoveTeleop;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignCoralStation extends Command {
  
  private SwerveSubsystem m_swerveSubsystem;
  private SwerveMoveTeleop m_moveCommand;

  public AlignCoralStation(
    SwerveSubsystem swerveSubsystem,
    SwerveMoveTeleop moveCommand
  ) {
    m_swerveSubsystem = swerveSubsystem;
    m_moveCommand = moveCommand;
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
    System.out.println("Gyro yaw: " + m_swerveSubsystem.getGlobalGyroAngle());
    if (m_swerveSubsystem.getGlobalGyroAngle() > 0) {
      m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(144));
      return;
    }
    
    m_moveCommand.setHeadingControl(Rotation2d.fromDegrees(-144));
  }

}
