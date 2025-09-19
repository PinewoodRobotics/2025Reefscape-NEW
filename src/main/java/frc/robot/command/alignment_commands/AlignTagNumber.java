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

  private final double rotationMultiplier = 0.5;
  private final double driveMultiplier = 0.1;
  private final double rotationThreshold = 6.0;
  private final double distanceThreshold = 0.1;

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
  }

  public AlignTagNumber(int tagNumber, Pose2d offset) {
    this.alignTagState = new AlignTagStateAutoLogged();

    this.alignTagState.setTagNumber(tagNumber);
    this.alignTagState.setOffset(offset);

    this.swerveSubsystem = SwerveSubsystem.GetInstance();
  }

  @Override
  public void initialize() {
    alignTagState.setAligning(true);
    alignTagState.setLatestTagData(AprilTagSubsystem.GetTagById(alignTagState.getTagNumber()));
    swerveSubsystem.drive(new Vec2(1, 0), 0, 0);
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

    swerveSubsystem.driveRaw(getDirectionToDrive(target),
        getOptimalDirectionRotate(alignTagState.getLatestTagData().getPose2d().getRotation(),
            alignTagState.getOffset().getRotation()) * rotationMultiplier,
        driveMultiplier);

    Logger.processInputs("AlignTagNumber", alignTagState);
  }

  @Override
  public boolean isFinished() {
    if (alignTagState.getLatestTagData() == null) {
      return false;
    }

    alignTagState.setDistanceRemaining(alignTagState.getTarget().getNorm());

    if (alignTagState.getDistanceRemaining() <= distanceThreshold
        && alignTagState.getRotationRemaining() <= rotationThreshold) {
      alignTagState.setAligning(false);
    } else {
      alignTagState.setAligning(true);
    }

    return !alignTagState.isAligning();
  }

  public static Vec2 getDirectionToDrive(Translation2d targetPose) {
    return new Vec2(-targetPose.getX(), targetPose.getY()).scaleToModulo(1);
  }

  public static Translation2d applyOffsetTranslation(Translation2d original, Translation2d offset) {
    return original.minus(offset);
  }

  public int getOptimalDirectionRotate(Rotation2d current, Rotation2d target) {
    // Use WPILib's built-in angle difference calculation which handles wraparound
    // correctly
    double angleDifference = target.minus(current).getRadians();
    alignTagState.setRotationRemaining(Math.abs(angleDifference));

    // Small threshold to avoid unnecessary micro-adjustments
    if (Math.abs(angleDifference) < Math.toRadians(1.0)) {
      return 0; // No rotation needed
    } else if (angleDifference > 0) {
      return -1; // Counterclockwise rotation
    } else {
      return 1; // Clockwise rotation
    }
  }
}
