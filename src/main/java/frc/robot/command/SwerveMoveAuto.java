package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath.EasingFunctions;
import frc.robot.util.position.RobotPosition2d;

public class SwerveMoveAuto extends Command {

  private final double kRotationSpeed = 0.9;

  private final SwerveSubsystem m_swerveSubsystem;

  private boolean isDone = false;
  private RobotPosition2d finalPosition;

  private double totalDistanceTarget = 0.0;
  private final double stopDistance;

  public SwerveMoveAuto(
      SwerveSubsystem swerveSubsystem,
      RobotPosition2d finalPosition,
      boolean isDone) {
    this(swerveSubsystem, finalPosition, 0.2, isDone);
  }

  public SwerveMoveAuto(
      SwerveSubsystem swerveSubsystem,
      RobotPosition2d finalPosition,
      double stopDistance,
      boolean isDone) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.finalPosition = finalPosition;
    this.isDone = isDone;
    this.stopDistance = stopDistance;
    addRequirements(m_swerveSubsystem);
  }

  public void setIsDone(boolean isDone) {
    this.isDone = isDone;
  }

  /**
   * @param finalPosition final position IN GLOBAL SYSTEM
   */
  public void setFinalPosition(RobotPosition2d finalPosition) {
    this.finalPosition = finalPosition;

    this.totalDistanceTarget = LocalizationSubsystem.getPose2d().getTranslation()
        .getDistance(finalPosition.getTranslation());
  }

  @Override
  public void execute() {
    if (isDone) {
      return;
    }

    var T_robot_world = LocalizationSubsystem.getPose2d().getSwerveRelative().getTransformationMatrix();
    var T_point_world = finalPosition.getSwerveRelative().getTransformationMatrix();
    var T_point_robot = T_robot_world.times(T_point_world.inv()).inv();

    Pose2d pointInRobot = extractFromTransformationMatrix(T_point_robot);

    int rotationDirection = rotationDirection(pointInRobot.getRotation(), finalPosition.getRotation());

    double dist = pointInRobot.getTranslation().getNorm();
    m_swerveSubsystem.driveRaw(
        new Vec2(pointInRobot.getX(), -pointInRobot.getY()),
        0, // rotationDirection * kRotationSpeed,
        EasingFunctions.easeOutCubic(0.0, totalDistanceTarget, dist, 0.15, 0.05));

    if (dist < stopDistance) {
      end(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    isDone = true;
    m_swerveSubsystem.drive(new Vec2(0, 0), 0, 0);
  }

  /**
     * Determines the rotation direction from the current rotation to the target rotation.
     * Returns -1 for a left (counterclockwise) turn, +1 for a right (clockwise) turn.
     *
     * @param current the current rotation
     * @param target the target rotation
     * @return -1 if the shortest path is a left turn, +1 if it is a right turn, 0 if aligned
     */
  private int rotationDirection(Rotation2d current, Rotation2d target) {
    double currentRad = current.getRadians();
    double targetRad = target.getRadians();

    double diff = targetRad - currentRad;

    diff = Math.atan2(Math.sin(diff), Math.cos(diff));

    if (diff > 0.1) {
      return -1;
    } else if (diff < -0.1) {
      return 1;
    } else {
      return 0;
    }
  }

  public Pose2d extractFromTransformationMatrix(Matrix<N3, N3> matrix) {
    double cosTheta = matrix.get(0, 0); // cos(theta)
    double sinTheta = matrix.get(1, 0); // sin(theta)
    double x = matrix.get(0, 2); // x translation
    double y = matrix.get(1, 2); // y translation

    return new Pose2d(x, y, new Rotation2d(cosTheta, sinTheta));
  }
}
