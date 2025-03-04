package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.pwrup.napoleon.bridge.AStarPathfinder;
import org.pwrup.napoleon.bridge.HybridGrid;
import org.pwrup.napoleon.bridge.NodePickStyle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.position.Orientation;
import frc.robot.util.position.RobotPosition2d;

public class PathfindingSubsystem extends Thread {

  private HybridGrid grid;
  private int[] destination;
  private int[][] latestPathOnMap;
  private Pose2d currentPosition;
  private double rerunDistanceThreshhold;
  private boolean isRerunNeeded = false;
  private AStarPathfinder aStarPathfinder;

  public PathfindingSubsystem(
      String mapFilePath,
      double rerunDistanceThreshhold,
      int maxNodesInRange,
      float robotWidth,
      float robotHeight) {
    this.grid = new HybridGrid(mapFilePath);
    this.rerunDistanceThreshhold = rerunDistanceThreshhold;
    this.aStarPathfinder = new AStarPathfinder(
        this.grid,
        NodePickStyle.SIDES,
        maxNodesInRange,
        robotWidth,
        robotHeight);
  }

  public void setHybridObstacles(float[][] positions) {
    aStarPathfinder.clearHybridObjects();
    addHybridObstacles(positions);
  }

  public void addHybridObstacles(float[][] positions) {
    float[] output = new float[positions.length * 2];
    int count = 0;
    for (float[] position : positions) {
      output[count] = position[0];
      output[count + 1] = position[1];
      count += 2;
    }

    this.isRerunNeeded = true;
  }

  public void clearHybridObstacles() {
    aStarPathfinder.clearHybridObjects();
  }

  public void setGoal(int[] position) {
    this.destination = position;
  }

  /**
   *
   * @param newPositionOnMap MUST BE IN MAP UNITS!! THIS IS NOT IN METERS!
   */
  public void updatePosition(Pose2d newPositionOnMap) {
    if (currentPosition
        .getTranslation()
        .getDistance(newPositionOnMap.getTranslation()) >= rerunDistanceThreshhold) {
      this.isRerunNeeded = true;
    }

    this.currentPosition = newPositionOnMap;
  }

  @Override
  public void run() {
    while (true) {
      if (!isRerunNeeded) {
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        continue;
      }

      int[] foundPath = aStarPathfinder.calculate(
          new int[] {
              (int) currentPosition.getX(),
              (int) currentPosition.getY(),
          },
          destination);

      int[][] latestPathOnMap = new int[foundPath.length / 2][2];
      int c = 0;
      for (int i = 0; i < foundPath.length; i += 2) {
        latestPathOnMap[c] = new int[] { foundPath[i], foundPath[i + 1] };
        c++;
      }
    }
  }

  public int[][] getMapLatestPath() {
    return this.latestPathOnMap;
  }

  public List<Pose2d> getGlobalLatestPath() {
    List<Pose2d> globalPath = new ArrayList<>();

    for (int[] point : latestPathOnMap) {
      double xMeters = (point[0] - grid.getCenterX()) * grid.getSqSizeMeters();
      double yMeters = (point[1] - grid.getCenterY()) * grid.getSqSizeMeters();

      globalPath.add(new Pose2d(xMeters, yMeters, new Rotation2d()));
    }

    return globalPath;
  }

  /**
   * 
   * @param currentPath current path without the rotatio components
   * @param start starting rotation of the robot in an ideal environment
   * @param end ending rotation of the robot
   * @return the path with integrated rotation component for each step
   * @usage essentially if you have a path gotten from a pathfinder, you want to incorporate a rotation component so that you can rotate smoothly. This is basically that.
   */
  public static List<RobotPosition2d> insertRotationComponent(
      List<Pose2d> currentPath,
      Rotation2d start,
      Rotation2d end) {
    List<RobotPosition2d> pathWithRotation = new ArrayList<>();

    double totalDistance = 0;
    for (int i = 0; i < currentPath.size() - 1; i++) {
      totalDistance += currentPath
          .get(i)
          .getTranslation()
          .getDistance(currentPath.get(i + 1).getTranslation());
    }

    double distanceTraveled = 0;
    for (int i = 0; i < currentPath.size(); i++) {
      if (i > 0) {
        distanceTraveled += currentPath
            .get(i - 1)
            .getTranslation()
            .getDistance(currentPath.get(i).getTranslation());
      }

      double t = distanceTraveled / totalDistance;

      Rotation2d interpolatedRotation = start.interpolate(end, t);

      pathWithRotation.add(
          new RobotPosition2d(
              currentPath.get(i).getX(),
              currentPath.get(i).getY(),
              interpolatedRotation,
              Orientation.FIELD));
    }

    return pathWithRotation;
  }
}
