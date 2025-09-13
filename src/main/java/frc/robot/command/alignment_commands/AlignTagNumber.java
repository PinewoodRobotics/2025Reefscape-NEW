package frc.robot.command.alignment_commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.TimedAprilTagData;
import lombok.Getter;
import lombok.Setter;

public class AlignTagNumber extends Command {

  private final double rotationMultiplier = 0.1;
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
    public TimedAprilTagData latestTagData;
    public double[] targetVector = new double[2];

    public boolean hasTagData = false;
    public boolean aligning = false;

    public double distanceRemaining = 0;
    public double rotationRemaining = 0;
    public int rotationDirection = 0;
  }

  /**
   * Aligns the robot to the tag number with an offset
   * 
   * @param tagNumber the tag number to align to
   * @param offset    the offset to align to. Essentially the tag with id is the
   *                  center axes where the x+ is pointing towards the tag
   */
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
    Translation2d target = applyOffset(tagTranslation, offset);

    alignTagState.setTarget(target);
    alignTagState.setTargetVector(new double[] { target.getX(), target.getY() });

    swerveSubsystem.driveRaw(getDirectionToDrive(target),
        0,
        driveMultiplier);

    Logger.processInputs("AlignTagNumber", alignTagState);
  }

  @Override
  public boolean isFinished() {
    if (alignTagState.getLatestTagData() == null) {
      return false;
    }

    alignTagState.setDistanceRemaining(alignTagState.getTarget().getNorm());

    if (alignTagState.getDistanceRemaining() <= distanceThreshold) {
      alignTagState.setAligning(false);
    } else {
      alignTagState.setAligning(true);
    }

    return !alignTagState.isAligning();
  }

  public static Vec2 getDirectionToDrive(Translation2d targetPose) {
    return new Vec2(-targetPose.getX(), targetPose.getY()).scaleToModulo(1);
  }

  public static Translation2d applyOffset(Translation2d original, Translation2d offset) {
    return original.plus(offset);
  }
}
