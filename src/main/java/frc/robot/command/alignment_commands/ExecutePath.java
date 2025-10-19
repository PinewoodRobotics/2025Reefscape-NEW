package frc.robot.command.alignment_commands;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import proto.pathfind.Pathfind.PathfindResult;

public class ExecutePath extends Command {
  private final List<Pose2d> path;

  public ExecutePath(List<Pose2d> path) {
    this.path = path;
    addRequirements(SwerveSubsystem.GetInstance());
  }

  public ExecutePath(PathfindResult pathfindResult) {
    this(pathfindResult.getPathList().stream()
        .map(path -> new Pose2d(path.getX(), path.getY(), new Rotation2d()))
        .collect(Collectors.toList()));
  }
}
