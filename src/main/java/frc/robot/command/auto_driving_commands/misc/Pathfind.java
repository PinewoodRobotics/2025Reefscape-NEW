package frc.robot.command.auto_driving_commands.misc;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlobalPosition;
import frc.robot.util.CustomMath;
import frc.robot.util.RPC;
import proto.pathfind.Pathfind.Algorithm;
import proto.pathfind.Pathfind.PathfindRequest;
import proto.pathfind.Pathfind.PathfindResult;
import proto.util.Vector.Vector2;

public class Pathfind extends Command {
  private final Translation2d start;
  private final Translation2d goal;
  private final Algorithm algorithm;
  private final boolean optimizePath;
  private CompletableFuture<PathfindResult> future;

  public Pathfind(Translation2d start, Translation2d goal, Algorithm algorithm, boolean optimizePath) {
    super();
    this.start = start;
    this.goal = goal;
    this.algorithm = algorithm;
    this.optimizePath = optimizePath;
  }

  public Pathfind(Pose2d goal, Algorithm algorithm, boolean optimizePath) {
    this(GlobalPosition.Get().getTranslation(), goal.getTranslation(), algorithm, optimizePath);
  }

  @Override
  public void initialize() {
    PathfindRequest request = PathfindRequest.newBuilder()
        .setStart(Vector2.newBuilder().setX((float) start.getX()).setY((float) start.getY()).build())
        .setGoal(Vector2.newBuilder().setX((float) goal.getX()).setY((float) goal.getY()).build())
        .setAlgorithm(algorithm)
        .setOptimizePath(optimizePath)
        .build();

    future = CompletableFuture.supplyAsync(() -> {
      return RPC.Services().pathfind(request);
    });
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return future != null && future.isDone();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      future.cancel(true);
    }
  }

  public Optional<PathfindResult> getResult() {
    PathfindResult result = null;

    try {
      result = future.get();
    } catch (InterruptedException | ExecutionException e) {
      e.printStackTrace();
      return Optional.empty();
    }

    return Optional.of(result);
  }

  public Optional<List<Translation2d>> getResultAsTranslation2dList() {
    var result = getResult();

    return result.isEmpty() ? Optional.empty()
        : Optional.of(CustomMath.fromPathfindResultToTranslation2dList(result.get()));
  }
}
