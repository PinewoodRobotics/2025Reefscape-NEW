package frc.robot.command.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.pwrup.util.Vec2;

public class DriveToPointOdometry extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final Pose2d m_targetPose;
  private final OdometrySubsystem m_odometrySubsystem;
  private boolean isEnabled;

  public DriveToPointOdometry(
    SwerveSubsystem m_swerveSubsystem,
    Pose2d m_targetPose,
    OdometrySubsystem m_odometrySubsystem,
    boolean isEnabled
  ) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.m_targetPose = m_targetPose;
    this.isEnabled = isEnabled;
    this.m_odometrySubsystem = m_odometrySubsystem;

    addRequirements(m_swerveSubsystem);
  }

  public void setEnabled(boolean isEnabled) {
    this.isEnabled = isEnabled;
  }

  @Override
  public void execute() {
    if (!isEnabled) {
      return;
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.isEnabled = false;
    m_swerveSubsystem.driveRaw(new Vec2(0, 0), 0, 0);
  }
}
