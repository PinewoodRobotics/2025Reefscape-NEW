package frc.robot.command.alignment_commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.TimedAprilTagData;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;
import pwrup.frc.core.geometry.CustomMath;

public class AlignTagNumber extends Command {

  @Setter
  @Getter
  @AutoLog
  @AllArgsConstructor
  private class AlignTagState {
    private int tagNumber;
    private Pose2d offset;
    private TimedAprilTagData latestTagData;
    private SwerveSubsystem swerveSubsystem;

    private boolean hasTagData;
    private boolean aligning;

    private double distanceRemaining;
    private double rotationRemaining;

    private final double rotationMultiplier = 0.3;
    private final double driveMultiplier = 0.05;

    private final double rotationThreshold = 6.0;
    private final double distanceThreshold = 0.1;
  }

  AlignTagState alignTagState;

  /**
   * Aligns the robot to the tag number with an offset
   * 
   * @param tagNumber the tag number to align to
   * @param offset    the offset to align to. Essentially the tag with id is the
   *                  center axes where the x+ is pointing towards the tag
   */
  public AlignTagNumber(int tagNumber, Pose2d offset) {
    this.alignTagState = new AlignTagState(tagNumber, offset, null, SwerveSubsystem.GetInstance(), false, false, 0, 0);
  }

  @Override
  public void initialize() {
    alignTagState.setAligning(true);
    alignTagState.setLatestTagData(AprilTagSubsystem.GetTagById(alignTagState.getTagNumber()));
    alignTagState.getSwerveSubsystem().drive(new Vec2(1, 0), 0, 0);
  }

  @Override
  public void execute() {
    var newTagData = AprilTagSubsystem.GetTagById(alignTagState.getTagNumber());

    alignTagState.setHasTagData(newTagData != null);
    if (!alignTagState.isHasTagData()) {
      return;
    }

    alignTagState.setLatestTagData(newTagData);

    alignTagState.getSwerveSubsystem().driveRaw(getDirectionToDrive(alignTagState.getLatestTagData().getPose2d()),
        frc.robot.util.CustomMath.getDirectionToRotate(alignTagState.getLatestTagData().getPose2d())
            * alignTagState.getRotationMultiplier(),
        alignTagState.getDriveMultiplier());
  }

  @Override
  public boolean isFinished() {
    if (alignTagState.getLatestTagData() == null) {
      return false;
    }

    alignTagState.setDistanceRemaining(alignTagState.getLatestTagData().getPose2d().getTranslation().getNorm());
    alignTagState
        .setRotationRemaining(Math.abs(alignTagState.getLatestTagData().getPose2d().getRotation().getDegrees()));

    alignTagState
        .setAligning(alignTagState.getRotationRemaining() <= alignTagState.getRotationThreshold()
            && alignTagState.getDistanceRemaining() <= alignTagState.getDistanceThreshold());

    return alignTagState.isAligning();
  }

  public static Vec2 getDirectionToDrive(Pose2d targetPose) {
    return new Vec2(targetPose.getX(), targetPose.getY()).scaleToModulo(1);
  }
}
