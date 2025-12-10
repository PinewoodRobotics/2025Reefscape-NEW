package frc.robot.command.auto_driving_commands.composite;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.command.auto_driving_commands.driving.ExecuteTrajectory;
import frc.robot.command.auto_driving_commands.misc.Pathfind;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.subsystems.SwerveSubsystem;
import proto.pathfind.Pathfind.Algorithm;

public class PathfindAndGo extends SequentialCommandGroup {

  public PathfindAndGo(Pose2d goal) {
    super();

    Pose2d start = GlobalPosition.Get();
    addRequirements(SwerveSubsystem.GetInstance());
    var pathfindCommand = new Pathfind(start.getTranslation(), goal.getTranslation(), Algorithm.ASTAR, true);
    addCommands(pathfindCommand, new ExecuteTrajectory(pathfindCommand.getResult().get()));
  }
}
