package frc.robot.command.alignment_commands;

import org.littletonrobotics.junction.Logger;
import org.pwrup.util.Vec2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.AprilTagSubsystem.AprilTagData;
import frc.robot.subsystems.SwerveSubsystem;
import pwrup.frc.core.geometry.CustomMath;

public class AlignTagNumber extends Command {

  private int tagNumber;
  private Pose2d offset;
  private AprilTagData latestTagData;
  private SwerveSubsystem m_swerveSubsystem;

  private final double rotationMultiplier = 0.3;
  private final double driveMultiplier = 0.05;

  /**
   * Aligns the robot to the tag number with an offset
   * 
   * @param tagNumber the tag number to align to
   * @param offset    the offset to align to. Essentially the tag with id is the
   *                  center axes where the x+ is pointing towards the tag
   */
  public AlignTagNumber(int tagNumber, Pose2d offset) {
    this.tagNumber = tagNumber;
    this.offset = offset;
    this.m_swerveSubsystem = SwerveSubsystem.GetInstance();
  }

  @Override
  public void initialize() {
    latestTagData = AprilTagSubsystem.GetTagById(tagNumber);
    m_swerveSubsystem.drive(new Vec2(1, 0), 0, 0);
  }

  @Override
  public void execute() {
    var newTagData = AprilTagSubsystem.GetTagById(tagNumber);
    if (newTagData == null) {
      System.out.println("No tag data found for tag number " + tagNumber);
      return;
    }

    System.out.println("!!!!!!!!!");

    latestTagData = newTagData;

    m_swerveSubsystem.drive(getDirectionToDrive(latestTagData.pose2d),
        getDirectionToRotate(latestTagData.pose2d) * rotationMultiplier, driveMultiplier);

    Logger.recordOutput("AlignTagNumber/LatestTagData/Distance", latestTagData.pose2d.getTranslation().getNorm());
    Logger.recordOutput("AlignTagNumber/LatestTagData/Pose", latestTagData.pose2d);
  }

  @Override
  public boolean isFinished() {
    if (latestTagData == null) {
      return false;
    }

    return withinThreshold(latestTagData.pose2d.getRotation().getDegrees(), 6.0)
        && latestTagData.pose2d.getTranslation().getNorm() <= 0.1;
  }

  public static boolean withinThreshold(double value, double threshold) {
    return CustomMath.applyMinimumThreshold(value, threshold) != 0;
  }

  public static Vec2 getDirectionToDrive(Pose2d targetPose) {
    return new Vec2(targetPose.getX(), targetPose.getY()).scaleToModulo(1);
  }

  /**
   * Returns the optimal direction to rotate to reach the target rotation.
   * 
   * @param targetPose The target pose to align to.
   * @return -1 if optimal to rotate counterclockwise, 1 if clockwise, 0 if
   *         already aligned.
   */
  public static int getDirectionToRotate(Pose2d targetPose) {
    double targetAngle = targetPose.getRotation().getDegrees();
    double currentAngle = SwerveSubsystem.GetInstance().getGlobalGyroAngle();

    double delta = ((targetAngle - currentAngle + 540) % 360) - 180;

    if (Math.abs(delta) < 1e-2) {
      return 0;
    }

    return delta > 0 ? 1 : -1;
  }
}
