package frc.robot.util.position;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.PathfindingSubsystem;

public class MostOptimalState {
  private PathfindingSubsystem pathfindingSubsystem;
  private double ballCost, tubeCost, pathMeterCost;

  class WeightedRobotState {
    double cost;
    RobotState state;

    public WeightedRobotState(double cost, RobotState state) {
      this.cost = cost;
      this.state = state;
    }
  }

  public MostOptimalState(PathfindingSubsystem pathfindingSubsystem, double ballCost, double tubeCost,
      double pathMeterCost) {
    this.pathfindingSubsystem = pathfindingSubsystem;
    this.ballCost = ballCost;
    this.tubeCost = tubeCost;
    this.pathMeterCost = pathMeterCost;
  }

  public WeightedRobotState findMostOptimalNextGoal(List<RobotState> robotStates, Pose2d currentPosition) {
    List<WeightedRobotState> addedCosts = new ArrayList<>();
    for (var state : robotStates) {
      double cost = 0;
      cost += state.futureTube ? tubeCost : 0;
      cost += state.futureBall ? ballCost : 0;

      int[][] path = pathfindingSubsystem.runSingular(currentPosition, state.position);
      cost += path.length * pathMeterCost;
      addedCosts.add(new WeightedRobotState(cost, state));
    }

    addedCosts.sort((a, b) -> {
      return Double.compare(a.cost, b.cost);
    });

    return addedCosts.get(0);
  }
}
