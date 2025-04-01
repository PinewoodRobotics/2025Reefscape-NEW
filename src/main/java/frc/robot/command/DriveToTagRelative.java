package frc.robot.command;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.ejml.simple.SimpleMatrix;
import org.pwrup.util.Vec2;

public class DriveToTagRelative extends Command {

  private final SwerveSubsystem m_swerveSubsystem;

  private double translationStoppingDistance;
  private double angularStoppingDistanceDeg;
  private double maxRotationSpeed;

  private boolean isDone = false;
  private Pose2d tag_in_camera_wanted;
  private long startTime = 0;
  private long maxTimeNoTagSeen;

  private int tagNumber;
  private double totalDistanceTarget;
  private double totalRotationTarget;
  private boolean isTotalDistanceTargetSet = false;

  public DriveToTagRelative(
    SwerveSubsystem swerveSubsystem,
    Pose2d finalPosition,
    int tagNumber,
    long maxTimeNoTagSeen,
    double translationStoppingDistance,
    double angularStoppingDistanceDeg,
    double maxRotationSpeed,
    boolean isDone
  ) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.tag_in_camera_wanted = finalPosition;
    this.tagNumber = tagNumber;
    this.maxTimeNoTagSeen = maxTimeNoTagSeen;
    this.translationStoppingDistance = translationStoppingDistance;
    this.angularStoppingDistanceDeg = angularStoppingDistanceDeg;
    this.maxRotationSpeed = maxRotationSpeed;
    this.isTotalDistanceTargetSet = false;
    addRequirements(m_swerveSubsystem);

    setIsDone(isDone);
  }

  public void setIsDone(boolean isDone) {
    this.isDone = isDone;
    if (!isDone) {
      startTime = System.currentTimeMillis();
      isTotalDistanceTargetSet = false;
    }
  }

  @Override
  public void execute() {
    /*if (isDone) {
      return;
    }

    var recentTags = AprilTagSubsystem.latestTagPositions;
    if (recentTags == null) {
      if (System.currentTimeMillis() - startTime > maxTimeNoTagSeen) {
        end(false);
      }

      return;
    }

    var allRecentTags = recentTags.positions;
    var tag_current = allRecentTags
        .stream()
        .filter(tag -> tag.tagNumber == tagNumber)
        .findFirst()
        .orElse(null);

    if (tag_current == null) {
      if (System.currentTimeMillis() - recentTags.timestamp > maxTimeNoTagSeen) {
        end(false);
      }

      return;
    }

    var tag_in_camera = tag_current.pose;
    var directionVec = worldToRobotFrame(tag_in_camera, tag_in_camera_wanted);
    var dist = directionVec.getNorm();
    double rotationDiff = getRotationDiff(tag_in_camera.getRotation(), tag_in_camera_wanted.getRotation());

    if (!isTotalDistanceTargetSet) {
      isTotalDistanceTargetSet = true;
      totalDistanceTarget = dist;
      totalRotationTarget = rotationDiff;
    }

    var convertedDirectionVec = new Vec2(
        (float) -directionVec.getY(),
        (float) -directionVec.getX());

    convertedDirectionVec.scaleToModulo(0.5);

    int rotationDirection = rotationDirection(rotationDiff, angularStoppingDistanceDeg);
    System.out.println(rotationDirection * maxRotationSpeed);
    m_swerveSubsystem.driveRaw(
        convertedDirectionVec,
        0/*rotationDirection * 0.4,
        0.1);

    if (dist < translationStoppingDistance) {
      end(false);
    }*/
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
  private int rotationDirection(
    Rotation2d current,
    Rotation2d target,
    double rangeDeg
  ) {
    return rotationDirection(getRotationDiff(current, target), rangeDeg);
  }

  private int rotationDirection(double diff, double rangeDeg) {
    if (diff > Math.toRadians(rangeDeg)) {
      return 1;
    } else if (diff < -Math.toRadians(rangeDeg)) {
      return -1;
    } else {
      return 0;
    }
  }

  private double getRotationDiff(Rotation2d current, Rotation2d target) {
    double currentRad = current.getRadians();
    double targetRad = target.getRadians();

    double diff = targetRad - currentRad;

    return Math.atan2(Math.sin(diff), Math.cos(diff));
  }

  public Pose2d extractFromTransformationMatrix(Matrix<N3, N3> matrix) {
    double cosTheta = matrix.get(0, 0); // cos(theta)
    double sinTheta = matrix.get(1, 0); // sin(theta)
    double x = matrix.get(0, 2); // x translation
    double y = matrix.get(1, 2); // y translation

    return new Pose2d(x, y, new Rotation2d(cosTheta, sinTheta));
  }

  /**
   * Converts a world-space point into the robot's local coordinate system.
   * The robot's local frame is defined as:
   * @x-axis +x = forward. Essentially the direction vector defines x-axis
   * @y-axis +y = left essentially the direction vector -90deg clockwise.
   *
   * @param robotPose  The robot's Pose2d in world coordinates.
   * @param targetPose The target point's Pose2d in world coordinates.
   * @return The target's position in the robot's local frame as a Translation2d.
   */
  Translation2d worldToRobotFrame(Pose2d robotPose, Pose2d targetPose) {
    var robotRotation = new SimpleMatrix(
      new double[][] {
        { robotPose.getRotation().getCos(), robotPose.getRotation().getSin() },
        { -robotPose.getRotation().getSin(), robotPose.getRotation().getCos() },
      }
    );

    var robotPositionGlobalFrame = new SimpleMatrix(
      new double[][] { { robotPose.getX() }, { robotPose.getY() } }
    );

    var targetPoseGlobalFrame = new SimpleMatrix(
      new double[][] { { targetPose.getX() }, { targetPose.getY() } }
    );

    SimpleMatrix displacementGlobal = targetPoseGlobalFrame.minus(
      robotPositionGlobalFrame
    );
    SimpleMatrix targetPoseLocalFrame = robotRotation.mult(displacementGlobal);

    return new Translation2d(
      (float) targetPoseLocalFrame.get(0, 0),
      (float) targetPoseLocalFrame.get(1, 0)
    );
  }
}
