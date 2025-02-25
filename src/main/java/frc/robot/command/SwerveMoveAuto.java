package frc.robot.command;

import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CustomMath.EasingFunctions;
import frc.robot.util.position.RobotPosition2d;
import frc.robot.util.position.RobotPositionType;

public class SwerveMoveAuto extends Command {

  private final double kRotationSpeed = 0.6;

  private final SwerveSubsystem m_swerveSubsystem;

  private boolean isDone = false;
  private RobotPosition2d finalPosition;

  private double totalDistanceTarget = 0.0;
  private final double stopDistance;

  public SwerveMoveAuto(
      SwerveSubsystem swerveSubsystem,
      RobotPosition2d finalPosition,
      boolean isDone) {
    this(swerveSubsystem, finalPosition, 0.5, isDone);
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
    assert finalPosition.positionType == RobotPositionType.GLOBAL;
    this.finalPosition = finalPosition;

    this.totalDistanceTarget = LocalizationSubsystem.getPose2d().getTranslation()
        .getDistance(finalPosition.getTranslation());
  }

  @Override
  public void execute() {
    if (isDone) {
      return;
    }

    // Get current pose
    RobotPosition2d currentPose = LocalizationSubsystem.getPose2d();

    // Compute robot relative frame (a weird thing between global and swerve relative...)
    // T_robot_global * T_pose_global = T_pose_robot
    RobotPosition2d globalRelativePose = new RobotPosition2d(finalPosition.relativeTo(currentPose),
        RobotPositionType.GLOBAL);
    RobotPosition2d localRelativePose = globalRelativePose.getSwerveRelative();

    int rotationDirection = rotationDirection(currentPose.getSwerveRelative().getRotation(),
        localRelativePose.getRotation());

    double dist = currentPose.getTranslation().getDistance(finalPosition.getTranslation());

    m_swerveSubsystem.driveRaw(
        new Vec2(localRelativePose.getX(), localRelativePose.getY()),
        rotationDirection * kRotationSpeed,
        EasingFunctions.easeOutCubic(0.0, totalDistanceTarget, dist, 0.5,
            0));

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

    if (diff > 0) {
      return -1;
    } else if (diff < 0) {
      return 1;
    } else {
      return 0;
    }
  }
}
