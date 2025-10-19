package frc.robot.command.alignment_commands;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.RPC;
import proto.pathfind.Pathfind.Algorithm;
import proto.pathfind.Pathfind.PathfindRequest;
import proto.pathfind.Pathfind.PathfindResult;
import proto.util.Vector.Vector2;

public class Pathfind extends Command {
  private final CompletableFuture<PathfindResult> future;
  private final Thread thread;

  public Pathfind(Translation2d start, Translation2d goal, Algorithm algorithm, boolean optimizePath) {
    super();
    future = new CompletableFuture<>();
    thread = new Thread(() -> {
      try {
        PathfindResult result = RPC.Services().pathfind(PathfindRequest.newBuilder()
            .setStart(Vector2.newBuilder().setX((float) start.getX()).setY((float) start.getY()).build())
            .setGoal(Vector2.newBuilder().setX((float) goal.getX()).setY((float) goal.getY()).build())
            .setAlgorithm(algorithm)
            .setOptimizePath(optimizePath)
            .build());
        future.complete(result);
      } catch (Exception e) {
        future.completeExceptionally(e);
      }
    });
  }

  @Override
  public void initialize() {
    thread.start();
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return future.isDone();
  }

  @Override
  public void end(boolean interrupted) {
    thread.interrupt();
  }

  public PathfindResult getResult() throws InterruptedException, ExecutionException {
    return future.get();
  }
}
