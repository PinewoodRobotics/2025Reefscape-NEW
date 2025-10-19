package frc.robot.command.alignment_commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.RPC;
import proto.pathfind.Pathfind.Algorithm;
import proto.pathfind.Pathfind.PathfindRequest;
import proto.pathfind.Pathfind.PathfindResult;
import proto.util.Vector.Vector2;

public class PathfindAndGo extends SequentialCommandGroup {

  public PathfindAndGo(Pose2d goal) {
    super();

    Pose2d start = GlobalPosition.Get();
    addRequirements(SwerveSubsystem.GetInstance());
    var pathfindCommand = new Pathfind(start.getTranslation(), goal.getTranslation(), Algorithm.ASTAR, true);
    addCommands(pathfindCommand);
  }
}
