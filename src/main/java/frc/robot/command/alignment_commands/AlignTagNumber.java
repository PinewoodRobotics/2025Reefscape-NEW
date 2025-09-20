package frc.robot.command.alignment_commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.camera.AprilTagSubsystem;
import frc.robot.util.TimedRobotCentricAprilTagData;
import lombok.Getter;
import lombok.Setter;

public class AlignTagNumber extends Command {

  // Proportional control constants
  private final double maxRotationSpeed = 0.3;
  private final double maxDriveSpeed = 0.1;
  private final double minRotationSpeed = 0.05;
  private final double minDriveSpeed = 0.02;

  // Thresholds for finishing alignment
  private final double rotationThreshold = 2.0;
  private final double distanceThreshold = 0.02;

  // Distance ranges for proportional control
  private final double maxRotationRange = 45.0; // degrees
  private final double maxDistanceRange = 1.0; // meters

  private final SwerveSubsystem swerveSubsystem;
  private AlignTagStateAutoLogged alignTagState;

  @Setter
  @Getter
  @AutoLog
  public static class AlignTagState {
    public int tagNumber = 0;
    public Pose2d offset = new Pose2d();
    public Translation2d target = new Translation2d();
    public TimedRobotCentricAprilTagData latestTagData;

    public boolean hasTagData = false;
    public boolean aligning = false;

    public double distanceRemaining = 0;
    public double rotationRemaining = 0;
    public int rotationDirection = 0;

    // Proportional control debugging values
    public double calculatedDriveSpeed = 0;
    public double calculatedRotationSpeed = 0;
  }

  public AlignTagNumber(int tagNumber, Pose2d offset) {
    this.alignTagState = new AlignTagStateAutoLogged();

    this.alignTagState.setOffset(offset);

    this.swerveSubsystem = SwerveSubsystem.GetInstance();
    addRequirements(swerveSubsystem);
  }

  public AlignTagNumber(Pose2d offset) {
    this(-100, offset);
  }

  @Override
  public void initialize() {
    alignTagState.setAligning(true);
    alignTagState.setLatestTagData(AprilTagSubsystem.GetTagById(alignTagState.getTagNumber()));
    var tag = AprilTagSubsystem.GetBestTag();
    if (tag == null) {
      alignTagState.setAligning(false);
      return;
    }

    this.alignTagState.setTagNumber(tag.getId());
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.driveRaw(new Vec2(0, 0), 0, 0);
    alignTagState.setAligning(false);
  }

  @Override
  public void execute() {
    var newTagData = AprilTagSubsystem.GetTagById(alignTagState.getTagNumber());

    alignTagState.setHasTagData(newTagData != null);
    if (!alignTagState.isHasTagData()) {
      return;
    }

    alignTagState.setLatestTagData(newTagData);

    Translation2d tagTranslation = alignTagState.getLatestTagData().getPose2d().getTranslation();
    Translation2d offset = alignTagState.getOffset().getTranslation();
    Translation2d target = applyOffsetTranslation(tagTranslation, offset);

    alignTagState.setTarget(target);

    // Calculate proportional speeds based on distance from target
    double distanceToTarget = target.getNorm();
    double driveSpeed = calculateProportionalDriveSpeed(distanceToTarget);

    double rotationError = Math.abs(alignTagState.getLatestTagData().getPose2d().getRotation()
        .minus(alignTagState.getOffset().getRotation()).getDegrees());
    double rotationSpeed = calculateProportionalRotationSpeed(rotationError);

    int rotationDirection = getOptimalDirectionRotate(alignTagState.getLatestTagData().getPose2d().getRotation(),
        alignTagState.getOffset().getRotation(), rotationThreshold);

    // Store calculated speeds for debugging
    alignTagState.setCalculatedDriveSpeed(driveSpeed);
    alignTagState.setCalculatedRotationSpeed(rotationSpeed);
    alignTagState.setRotationDirection(rotationDirection);

    swerveSubsystem.driveRaw(SwerveSubsystem.toSwerveOrientation(target),
        rotationDirection * rotationSpeed,
        driveSpeed);

    Logger.processInputs("AlignTagNumber", alignTagState);
  }

  @Override
  public boolean isFinished() {
    if (alignTagState.getLatestTagData() == null) {
      return false;
    }

    alignTagState.setDistanceRemaining(alignTagState.getTarget().getNorm());
    alignTagState.setRotationRemaining(alignTagState.getLatestTagData().getPose2d().getRotation()
        .minus(alignTagState.getOffset().getRotation()).getDegrees());

    if (alignTagState.getDistanceRemaining() <= distanceThreshold
        && alignTagState.getRotationRemaining() <= rotationThreshold) {
      alignTagState.setAligning(false);
    } else {
      alignTagState.setAligning(true);
    }

    return !alignTagState.isAligning();
  }

  public static Translation2d applyOffsetTranslation(Translation2d original, Translation2d offset) {
    return original.minus(offset);
  }

  public int getOptimalDirectionRotate(Rotation2d current, Rotation2d target, double thresholdDegrees) {
    double angleDifference = target.minus(current).getRadians();
    alignTagState.setRotationRemaining(Math.abs(angleDifference));

    if (Math.abs(angleDifference) < Math.toRadians(thresholdDegrees)) {
      return 0;
    } else if (angleDifference > 0) {
      return -1;
    } else {
      return 1;
    }
  }

  /**
   * Calculates proportional drive speed based on distance to target
   * 
   * @param distance Distance to target in meters
   * @return Speed value between minDriveSpeed and maxDriveSpeed
   */
  private double calculateProportionalDriveSpeed(double distance) {
    if (distance <= distanceThreshold) {
      return 0.0;
    }

    // Linear interpolation between min and max speed
    double normalizedDistance = Math.min(distance / maxDistanceRange, 1.0);
    double speed = minDriveSpeed + (maxDriveSpeed - minDriveSpeed) * normalizedDistance;

    return Math.max(minDriveSpeed, Math.min(maxDriveSpeed, speed));
  }

  /**
   * Calculates proportional rotation speed based on rotation error
   * 
   * @param rotationError Rotation error in degrees
   * @return Speed value between minRotationSpeed and maxRotationSpeed
   */
  private double calculateProportionalRotationSpeed(double rotationError) {
    if (rotationError <= rotationThreshold) {
      return 0.0;
    }

    // Linear interpolation between min and max speed
    double normalizedError = Math.min(rotationError / maxRotationRange, 1.0);
    double speed = minRotationSpeed + (maxRotationSpeed - minRotationSpeed) * normalizedError;

    return Math.max(minRotationSpeed, Math.min(maxRotationSpeed, speed));
  }
}
