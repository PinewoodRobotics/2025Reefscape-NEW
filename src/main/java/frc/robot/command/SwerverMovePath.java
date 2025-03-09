package frc.robot.command;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.position.RobotPosition2d;

public class SwerverMovePath extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private LinkedList<RobotPosition2d> waypoints;
  private SwerveMoveAuto currentCommand;

  public SwerverMovePath(
      SwerveSubsystem swerveSubsystem,
      List<RobotPosition2d> pathList) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.waypoints = new LinkedList<>(pathList);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    scheduleNextPoint();
  }

  public void setPathGoal(List<RobotPosition2d> pathList) {
    this.currentCommand.end(true);
    this.waypoints = new LinkedList<>(pathList);
    initialize();
  }

  private void scheduleNextPoint() {
    if (!waypoints.isEmpty()) {
      RobotPosition2d nextPoint = waypoints.poll();
      currentCommand = new SwerveMoveAuto(this.m_swerveSubsystem, null, nextPoint, false);
      CommandScheduler.getInstance().schedule(currentCommand);
    }
  }

  @Override
  public void execute() {
    if (currentCommand != null && currentCommand.isFinished()) {
      scheduleNextPoint();
    }
  }

  @Override
  public boolean isFinished() {
    return waypoints.isEmpty() && (currentCommand == null || currentCommand.isFinished());
  }

  @Override
  public void end(boolean interrupted) {
    if (currentCommand != null || !currentCommand.isFinished()) {
      currentCommand.end(true);
    }
  }
}
